%% FastSLAM 1.0 vs EKF SLAM comparison 

%    1. Legend uses explicit gobject handle array — correct colours
%    2. Comparison panel uses two named lines on single axis (no yyaxis)
%    3. Particle subsample display capped at 25 for visual clarity
%    4. RMSE running averages smoothed for cleaner plot

clear; clc; close all;
rng(42);

%% ── Parameters ───────────────────────────────────────────────────────────
DT=0.1; N_STEPS=300; LIDAR_RANGE=3.5; WORLD_SIZE=9;
SIG_V=0.08; SIG_W=0.04; SIG_R=0.06; SIG_PHI=0.03;
Q_robot  = diag([SIG_V^2, SIG_V^2, SIG_W^2]);
R_sensor = diag([SIG_R^2, SIG_PHI^2]);
ASSOC_TH = 0.85;
N_PART   = 50;

true_lm=[2,2;4,1;5,3;3,5;1,4;6,2;7,4;5,6;2,7;7,7;0.5,6;4,8;8,1;8,5;1,1];
waypts=[1,1;7,1;7,8;1,8;1,1;4,4;7,1;1,8;7,8;1,1];
wp_idx=1;

%% ── Init ─────────────────────────────────────────────────────────────────
init_pose=[0.5;0.5;pi/4];
particles=init_particles(N_PART,init_pose,Q_robot*0.1);
ekf_mu=init_pose; ekf_Sigma=zeros(3,3);
ekf_reg=containers.Map('KeyType','int32','ValueType','int32');
ts=init_pose;

tp=zeros(N_STEPS+1,2); tp(1,:)=ts(1:2)';
fp=zeros(N_STEPS+1,2); fp(1,:)=init_pose(1:2)';
ep=zeros(N_STEPS+1,2); ep(1,:)=init_pose(1:2)';
fs_err=zeros(N_STEPS,1); ekf_err=zeros(N_STEPS,1);

%% ── Figure ───────────────────────────────────────────────────────────────
fig=figure('Color','w','Position',[60 60 1100 520],'Renderer','painters');
tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
ax_fs=nexttile(tl); hold(ax_fs,'on'); axis(ax_fs,'equal');
ax_cmp=nexttile(tl); hold(ax_cmp,'on');

fprintf('FastSLAM vs EKF SLAM (%d particles)...\n',N_PART);

%% ── Main loop ────────────────────────────────────────────────────────────
for t=1:N_STEPS
    [v,w]=ctrl(ts,waypts,wp_idx);
    if norm(ts(1:2)-waypts(wp_idx,:)')<0.3
        wp_idx=mod(wp_idx,size(waypts,1))+1;
    end
    ts=mmove(ts,v+randn*SIG_V,w+randn*SIG_W,DT);
    [obs,rays]=do_lidar(ts,true_lm,LIDAR_RANGE,SIG_R,SIG_PHI);

    % FastSLAM
    particles=fs_update(particles,v,w,DT,obs,Q_robot,R_sensor,ASSOC_TH);
    particles=resample(particles);
    fs_mean=weighted_mean(particles);

    % EKF SLAM
    [ekf_mu,ekf_Sigma]=ekf_pred(ekf_mu,ekf_Sigma,v,w,DT,Q_robot);
    [ekf_mu,ekf_Sigma,ekf_reg]=ekf_upd(ekf_mu,ekf_Sigma,obs,ekf_reg,R_sensor,ASSOC_TH);

    tp(t+1,:)=ts(1:2)'; fp(t+1,:)=fs_mean(1:2)'; ep(t+1,:)=ekf_mu(1:2)';
    fs_err(t)=norm(ts(1:2)-fs_mean(1:2));
    ekf_err(t)=norm(ts(1:2)-ekf_mu(1:2));

    if mod(t,3)==0
        draw_fs(ax_fs,ax_cmp,ts,particles,fs_mean,ekf_mu, ...
            tp(1:t+1,:),fp(1:t+1,:),ep(1:t+1,:), ...
            true_lm,rays,fs_err(1:t),ekf_err(1:t),t,N_STEPS,WORLD_SIZE);
        drawnow limitrate;
    end
end

fprintf('Final RMSE — FastSLAM: %.4f m   EKF: %.4f m\n', ...
    sqrt(mean(fs_err.^2)),sqrt(mean(ekf_err.^2)));

%% ══════════════════════════════════════════════════════════════════════════
%% FASTSLAM CORE
%% ══════════════════════════════════════════════════════════════════════════
function pts=init_particles(M,pose,Q)
    pts=struct('pose',{},'lm',{},'w',{});
    for i=1:M
        p.pose=pose+chol(Q)'*randn(3,1); p.pose(3)=wangle(p.pose(3));
        p.lm={}; p.w=1/M; pts(i)=p;
    end
end

function pts=fs_update(pts,v,w,dt,obs,Qr,R,th)
    M=length(pts);
    for i=1:M
        noise=chol(Qr)'*randn(3,1);
        pts(i).pose=mmove(pts(i).pose,v+noise(1),w+noise(3),dt);
        if isempty(obs),continue;end
        log_w=0;
        for k=1:size(obs,1)
            ro=obs(k,2); po=obs(k,3);
            best=-1; bd=th*4; best_lh=-inf;
            for j=1:length(pts(i).lm)
                if isempty(pts(i).lm{j}),continue;end
                zh=zhat2(pts(i).pose,pts(i).lm{j}.mu);
                inn=[ro-zh(1);wangle(po-zh(2))];
                H=lm_jac(pts(i).pose,pts(i).lm{j}.mu);
                Sz=H*pts(i).lm{j}.Sigma*H'+R;
                d=inn'*(Sz\inn);
                if d<bd
                    bd=d; best=j;
                    best_lh=-0.5*(log(max(det(2*pi*Sz),1e-12))+d);
                end
            end
            if best>0
                H=lm_jac(pts(i).pose,pts(i).lm{best}.mu);
                zh=zhat2(pts(i).pose,pts(i).lm{best}.mu);
                inn=[ro-zh(1);wangle(po-zh(2))];
                Sz=H*pts(i).lm{best}.Sigma*H'+R;
                K=pts(i).lm{best}.Sigma*H'/Sz;
                pts(i).lm{best}.mu=pts(i).lm{best}.mu+K*inn;
                pts(i).lm{best}.Sigma=(eye(2)-K*H)*pts(i).lm{best}.Sigma;
                log_w=log_w+best_lh;
            else
                rx=pts(i).pose(1);ry=pts(i).pose(2);rth=pts(i).pose(3);
                lm_new.mu=[rx+ro*cos(po+rth);ry+ro*sin(po+rth)];
                lm_new.Sigma=eye(2)*0.5;
                pts(i).lm{end+1}=lm_new;
                log_w=log_w-2;
            end
        end
        pts(i).w=pts(i).w*exp(log_w);
    end
    ws=[pts.w]; s=sum(ws);
    if s<1e-300,ws=ones(1,M)/M;end
    ws=ws/sum(ws);
    for i=1:M,pts(i).w=ws(i);end
end

function pts=resample(pts)
    M=length(pts); ws=[pts.w];
    if 1/sum(ws.^2)>M/2,return;end
    c=ws(1);i=1;idx=zeros(1,M);u=rand/M;
    for j=1:M
        uu=u+(j-1)/M;
        while uu>c&&i<M,i=i+1;c=c+ws(i);end
        idx(j)=i;
    end
    pts=pts(idx);
    for i=1:M,pts(i).w=1/M;end
end

function mu=weighted_mean(pts)
    ws=[pts.w]; mu=zeros(3,1);
    for i=1:length(pts),mu(1:2)=mu(1:2)+ws(i)*pts(i).pose(1:2);end
    s=0;c=0;
    for i=1:length(pts),s=s+ws(i)*sin(pts(i).pose(3));c=c+ws(i)*cos(pts(i).pose(3));end
    mu(3)=atan2(s,c);
end

function z=zhat2(pose,lm),dx=lm(1)-pose(1);dy=lm(2)-pose(2);
    z=[sqrt(dx^2+dy^2);wangle(atan2(dy,dx)-pose(3))];end
function H=lm_jac(pose,lm),dx=lm(1)-pose(1);dy=lm(2)-pose(2);
    r2=dx^2+dy^2;r=sqrt(r2);H=[dx/r,dy/r;-dy/r2,dx/r2];end

%% ══════════════════════════════════════════════════════════════════════════
%% DRAW — explicit handles, no yyaxis
%% ══════════════════════════════════════════════════════════════════════════
function draw_fs(axf,axc,ts,pts,fs_mean,ekf_mu, ...
        tp,fp,ep,lms,rays,fse,ekfe,t,nt,ws)

    %% Map panel
    cla(axf); hold(axf,'on'); axis(axf,'equal',[0 ws 0 ws]); grid(axf,'on');
    set(axf,'FontSize',10,'Box','on');

    for i=1:size(rays,1)
        line(axf,[rays(i,1) rays(i,3)],[rays(i,2) rays(i,4)], ...
            'Color',[0 0.74 0.84 0.20],'LineWidth',0.6,'HandleVisibility','off');
    end

    h(1)=plot(axf,tp(:,1),tp(:,2),'-','Color',[0.18 0.69 0.18],'LineWidth',1.5);
    h(2)=plot(axf,fp(:,1),fp(:,2),'-','Color',[0.60 0.10 0.80],'LineWidth',1.5);
    h(3)=plot(axf,ep(:,1),ep(:,2),'--','Color',[0.15 0.42 0.90],'LineWidth',1.2);
    h(4)=plot(axf,NaN,NaN,'^','Color',[1.00 0.55 0.00], ...
        'MarkerFaceColor',[1.00 0.55 0.00],'MarkerSize',8);
    plot(axf,lms(:,1),lms(:,2),'^','Color',[1.00 0.55 0.00], ...
        'MarkerFaceColor',[1.00 0.55 0.00],'MarkerSize',8,'HandleVisibility','off');

    % Particles (subsample max 25)
    skip=max(1,floor(length(pts)/25));
    h(5)=plot(axf,NaN,NaN,'.','Color',[0.60 0.10 0.80],'MarkerSize',6);
    for i=1:skip:length(pts)
        plot(axf,pts(i).pose(1),pts(i).pose(2),'.', ...
            'Color',[0.60 0.10 0.80 0.60],'MarkerSize',5,'HandleVisibility','off');
    end

    % Best particle landmarks
    [~,bi]=max([pts.w]);
    plotted_lm=false;
    for j=1:length(pts(bi).lm)
        if isempty(pts(bi).lm{j}),continue;end
        mx=pts(bi).lm{j}.mu(1); my=pts(bi).lm{j}.mu(2);
        if ~plotted_lm
            h(6)=plot(axf,mx,my,'p','Color',[0.85 0.10 0.40], ...
                'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',7);
            plotted_lm=true;
        else
            plot(axf,mx,my,'p','Color',[0.85 0.10 0.40], ...
                'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',7,'HandleVisibility','off');
        end
    end
    if ~plotted_lm
        h(6)=plot(axf,NaN,NaN,'p','Color',[0.85 0.10 0.40], ...
            'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',7);
    end

    % Robot dots
    plot(axf,ts(1),ts(2),'o','Color',[0.18 0.69 0.18], ...
        'MarkerFaceColor',[0.18 0.69 0.18],'MarkerSize',10,'HandleVisibility','off');
    plot(axf,fs_mean(1),fs_mean(2),'o','Color',[0.60 0.10 0.80], ...
        'MarkerFaceColor',[0.60 0.10 0.80],'MarkerSize',9,'HandleVisibility','off');
    plot(axf,ekf_mu(1),ekf_mu(2),'o','Color',[0.15 0.42 0.90], ...
        'MarkerFaceColor',[0.15 0.42 0.90],'MarkerSize',8,'HandleVisibility','off');

    legend(axf,h,{'True path','FastSLAM path','EKF path','True LM', ...
        sprintf('Particles (n=%d)',length(pts)),'Est. LM (best)'}, ...
        'Location','northeast','FontSize',8,'Box','on');
    xlabel(axf,'x [m]'); ylabel(axf,'y [m]');
    title(axf,sprintf('FastSLAM vs EKF SLAM  |  step %d',t), ...
        'FontSize',11,'FontWeight','normal');

    %% Comparison panel — two lines on single axis
    cla(axc); hold(axc,'on'); grid(axc,'on');
    set(axc,'FontSize',10,'Box','on');
    steps=1:t;

    % Smooth with 10-step moving average for cleaner plot
    win=min(10,t);
    fse_sm=movmean(fse,win); ekfe_sm=movmean(ekfe,win);

    hc1=plot(axc,steps,fse_sm,'Color',[0.60 0.10 0.80],'LineWidth',2.0);
    hc2=plot(axc,steps,ekfe_sm,'--','Color',[0.15 0.42 0.90],'LineWidth',2.0);

    legend(axc,[hc1,hc2], ...
        {sprintf('FastSLAM RMSE=%.3f m',sqrt(mean(fse.^2))), ...
         sprintf('EKF SLAM  RMSE=%.3f m',sqrt(mean(ekfe.^2)))}, ...
        'Location','northeast','FontSize',9,'Box','on');
    xlabel(axc,'Time step'); ylabel(axc,'Pose error [m]');
    title(axc,'Pose error comparison (10-step smoothed)', ...
        'FontSize',11,'FontWeight','normal');
    xlim(axc,[0 nt]);
    ylim(axc,[0 max([fse;ekfe])*1.4+0.01]);
end

%% ── Shared helpers ───────────────────────────────────────────────────────
function s=mmove(s,v,w,dt)
    th=s(3);s(1)=s(1)+v*cos(th+w*dt/2)*dt;
    s(2)=s(2)+v*sin(th+w*dt/2)*dt;s(3)=wangle(s(3)+w*dt);
end
function [mu,Sigma]=ekf_pred(mu,Sigma,v,w,dt,Qr)
    th=mu(3);n=length(mu);
    mu(1)=mu(1)+v*cos(th+w*dt/2)*dt;mu(2)=mu(2)+v*sin(th+w*dt/2)*dt;
    mu(3)=wangle(mu(3)+w*dt);
    G=eye(n);G(1,3)=-v*sin(th+w*dt/2)*dt;G(2,3)=v*cos(th+w*dt/2)*dt;
    Qf=zeros(n);Qf(1:3,1:3)=Qr;Sigma=G*Sigma*G'+Qf;
end
function [obs,rays]=do_lidar(st,lms,rng,sr,sp)
    obs=[];rays=[];
    for i=1:size(lms,1)
        dx=lms(i,1)-st(1);dy=lms(i,2)-st(2);r=sqrt(dx^2+dy^2);
        if r>rng,continue;end
        phi=wangle(atan2(dy,dx)-st(3));
        obs=[obs;i,r+randn*sr,wangle(phi+randn*sp)];%#ok
        rays=[rays;st(1),st(2),lms(i,1),lms(i,2)];%#ok
    end
end
function [mu,Sigma,reg]=ekf_upd(mu,Sigma,obs,reg,R,th)
    if isempty(obs),return;end
    nL=(length(mu)-3)/2;
    for k=1:size(obs,1)
        tid=obs(k,1);ro=obs(k,2);po=obs(k,3);
        if isKey(reg,int32(tid)),idx=reg(int32(tid));
        else
            best=-1;bd=th;
            for j=0:nL-1
                mi=3+j*2+1;zh=zhat(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
                inn=[ro-zh(1);wangle(po-zh(2))];d=sqrt(inn(1)^2+inn(2)^2*4);
                if d<bd,bd=d;best=j;end
            end
            if best>=0,idx=best;reg(int32(tid))=idx;
            else
                mu=[mu;mu(1)+ro*cos(po+mu(3));mu(2)+ro*sin(po+mu(3))];
                n=length(mu);Sn=zeros(n);Sn(1:n-2,1:n-2)=Sigma;
                Sn(n-1,n-1)=1;Sn(n,n)=1;Sigma=Sn;idx=nL;nL=nL+1;reg(int32(tid))=idx;
            end
        end
        mi=3+idx*2+1;zh=zhat(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
        inn=[ro-zh(1);wangle(po-zh(2))];
        Hs=Hjac(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
        n=length(mu);H=zeros(2,n);H(:,1:3)=Hs(:,1:3);H(:,mi:mi+1)=Hs(:,4:5);
        S=H*Sigma*H'+R;K=Sigma*H'/S;mu=mu+K*inn;mu(3)=wangle(mu(3));
        Sigma=(eye(n)-K*H)*Sigma;
    end
end
function z=zhat(rx,ry,rth,mx,my)
    dx=mx-rx;dy=my-ry;z=[sqrt(dx^2+dy^2);wangle(atan2(dy,dx)-rth)];end
function H=Hjac(rx,ry,~,mx,my)
    dx=mx-rx;dy=my-ry;r2=dx^2+dy^2;r=sqrt(r2);
    H=[-dx/r,-dy/r,0,dx/r,dy/r;dy/r2,-dx/r2,-1,-dy/r2,dx/r2];end
function [v,w]=ctrl(st,wp,i)
    dx=wp(i,1)-st(1);dy=wp(i,2)-st(2);
    v=min(0.5,sqrt(dx^2+dy^2))*0.8;w=wangle(atan2(dy,dx)-st(3))*2;end
function a=wangle(a),a=mod(a+pi,2*pi)-pi;end