%% GIF export for EKF SLAM 
% Saves two GIFs to the current folder:
%   ekf_slam_map.gif
%   ekf_slam_metrics.gif
%
% FIX vs previous version:
%   Uses print() -> tmpPNG -> imread -> imwrite chain.
%   getframe() silently fails with OpenGL renderer in R2024a.

clear; clc; close all;
rng(42);

%% ── GIF settings ─────────────────────────────────────────────────────────
GIF_DELAY  = 0.07;
GIF_SKIP   = 4;
OUT_MAP    = 'ekf_slam_map.gif';
OUT_MET    = 'ekf_slam_metrics.gif';
TMP_MAP    = fullfile(tempdir, 'slam_map_tmp.png');
TMP_MET    = fullfile(tempdir, 'slam_met_tmp.png');

%% ── Sim params ───────────────────────────────────────────────────────────
DT=0.1; N_STEPS=300; LIDAR_RANGE=3.5; WORLD_SIZE=9;
SIG_V=0.08; SIG_W=0.04; SIG_R=0.06; SIG_PHI=0.03;
Q_robot  = diag([SIG_V^2, SIG_V^2, SIG_W^2]);
R_sensor = diag([SIG_R^2, SIG_PHI^2]);

true_lm=[2,2;4,1;5,3;3,5;1,4;6,2;7,4;5,6;2,7;7,7;0.5,6;4,8;8,1;8,5;1,1];
waypts=[1,1;7,1;7,8;1,8;1,1;4,4;7,1;1,8;7,8;1,1];
wp_idx=1;

ts=[0.5;0.5;pi/4]; mu=[0.5;0.5;pi/4]; Sigma=zeros(3,3);
lm_reg=containers.Map('KeyType','int32','ValueType','int32');
tp=zeros(N_STEPS+1,2); tp(1,:)=ts(1:2)';
ep=zeros(N_STEPS+1,2); ep(1,:)=mu(1:2)';
pe=zeros(N_STEPS,1); nl=zeros(N_STEPS,1);

%% ── Figures with painters renderer ──────────────────────────────────────
fig_map=figure('Color','w','Position',[60 60 780 620], ...
    'Renderer','painters','MenuBar','none','ToolBar','none');
ax_map=axes(fig_map,'FontSize',10); hold(ax_map,'on'); axis(ax_map,'equal');

fig_met=figure('Color','w','Position',[900 60 780 360], ...
    'Renderer','painters','MenuBar','none','ToolBar','none');
ax_met=axes(fig_met,'FontSize',10); hold(ax_met,'on');

fprintf('Generating GIFs...\n');
first_map=true; first_met=true;

for t=1:N_STEPS
    [v,w]=ctrl(ts,waypts,wp_idx);
    if norm(ts(1:2)-waypts(wp_idx,:)')<0.3
        wp_idx=mod(wp_idx,size(waypts,1))+1;
    end
    ts=mmove(ts,v+randn*SIG_V,w+randn*SIG_W,DT);
    [mu,Sigma]=ekf_pred(mu,Sigma,v,w,DT,Q_robot);
    [obs,rays]=do_lidar(ts,true_lm,LIDAR_RANGE,SIG_R,SIG_PHI);
    [mu,Sigma,lm_reg]=ekf_upd(mu,Sigma,obs,lm_reg,R_sensor,0.8);
    tp(t+1,:)=ts(1:2)'; ep(t+1,:)=mu(1:2)';
    pe(t)=norm(ts(1:2)-mu(1:2)); nl(t)=(length(mu)-3)/2;

    if mod(t,GIF_SKIP)~=0, continue; end

    % ── Map frame ──────────────────────────────────────────────────────
    draw_map(ax_map,ts,mu,Sigma,tp(1:t+1,:),ep(1:t+1,:), ...
             true_lm,rays,WORLD_SIZE,t);
    drawnow; pause(0.02);
    print(fig_map,TMP_MAP,'-dpng','-r96');
    [im,cm]=rgb2ind(imread(TMP_MAP),128);
    if first_map
        imwrite(im,cm,OUT_MAP,'gif','Loopcount',inf,'DelayTime',GIF_DELAY);
        first_map=false;
    else
        imwrite(im,cm,OUT_MAP,'gif','WriteMode','append','DelayTime',GIF_DELAY);
    end

    % ── Metrics frame ──────────────────────────────────────────────────
    draw_met(ax_met,pe(1:t),nl(1:t),t,N_STEPS);
    drawnow; pause(0.02);
    print(fig_met,TMP_MET,'-dpng','-r96');
    [im,cm]=rgb2ind(imread(TMP_MET),128);
    if first_met
        imwrite(im,cm,OUT_MET,'gif','Loopcount',inf,'DelayTime',GIF_DELAY);
        first_met=false;
    else
        imwrite(im,cm,OUT_MET,'gif','WriteMode','append','DelayTime',GIF_DELAY);
    end

    fprintf('  step %3d | LMs: %d | err: %.3f m\n',t,nl(t),pe(t));
end
if isfile(TMP_MAP),delete(TMP_MAP);end
if isfile(TMP_MET),delete(TMP_MET);end
fprintf('Done.\n  %s\n  %s\n',OUT_MAP,OUT_MET);

%% ── Draw map ─────────────────────────────────────────────────────────────
function draw_map(ax,ts,mu,Sigma,tp,ep,lms,rays,ws,t)
    cla(ax); hold(ax,'on'); axis(ax,'equal',[0 ws 0 ws]); grid(ax,'on');
    set(ax,'FontSize',10,'Box','on');

    h=gobjects(5,1);
    for i=1:size(rays,1)
        line(ax,[rays(i,1) rays(i,3)],[rays(i,2) rays(i,4)], ...
            'Color',[0 0.74 0.84 0.30],'LineWidth',0.7,'HandleVisibility','off');
    end
    h(1)=plot(ax,NaN,NaN,'-','Color',[0 0.74 0.84],'LineWidth',1.2); % lidar dummy
    h(2)=plot(ax,tp(:,1),tp(:,2),'-','Color',[0.18 0.69 0.18],'LineWidth',1.5);
    h(3)=plot(ax,ep(:,1),ep(:,2),'-','Color',[0.15 0.42 0.90],'LineWidth',1.5);
    h(4)=plot(ax,lms(:,1),lms(:,2),'^','Color',[1 0.55 0], ...
        'MarkerFaceColor',[1 0.55 0],'MarkerSize',9,'LineWidth',0.5);

    n_lm=(length(mu)-3)/2;
    for j=0:n_lm-1
        mi=3+j*2+1; mx=mu(mi); my=mu(mi+1);
        if j==0
            h(5)=plot(ax,mx,my,'p','Color',[0.85 0.10 0.40], ...
                'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',8);
        else
            plot(ax,mx,my,'p','Color',[0.85 0.10 0.40], ...
                'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',8, ...
                'HandleVisibility','off');
        end
        Slm=Sigma(mi:mi+1,mi:mi+1);
        [V,D]=eig((Slm+Slm')/2);
        ang=linspace(0,2*pi,50);
        e=3*V*sqrt(max(D,0))*[cos(ang);sin(ang)];
        plot(ax,mx+e(1,:),my+e(2,:),'--','Color',[0.55 0.55 0.55 0.50], ...
            'LineWidth',0.7,'HandleVisibility','off');
    end

    plot(ax,ts(1),ts(2),'o','Color',[0.18 0.69 0.18], ...
        'MarkerFaceColor',[0.18 0.69 0.18],'MarkerSize',10,'HandleVisibility','off');
    quiver(ax,ts(1),ts(2),0.3*cos(ts(3)),0.3*sin(ts(3)),0, ...
        'Color',[0.18 0.69 0.18],'LineWidth',2,'MaxHeadSize',2,'HandleVisibility','off');
    plot(ax,mu(1),mu(2),'o','Color',[0.15 0.42 0.90], ...
        'MarkerFaceColor',[0.15 0.42 0.90],'MarkerSize',9,'HandleVisibility','off');
    quiver(ax,mu(1),mu(2),0.3*cos(mu(3)),0.3*sin(mu(3)),0, ...
        'Color',[0.15 0.42 0.90],'LineWidth',2,'MaxHeadSize',2,'HandleVisibility','off');
    Se=Sigma(1:2,1:2); [V,D]=eig((Se+Se')/2);
    ang=linspace(0,2*pi,50);
    e=3*V*sqrt(max(D,0))*[cos(ang);sin(ang)];
    plot(ax,mu(1)+e(1,:),mu(2)+e(2,:),'-','Color',[0.15 0.42 0.90 0.45], ...
        'LineWidth',1,'HandleVisibility','off');

    legend(ax,h,{'LiDAR ray','True path','EKF path','True LM','Est. LM'}, ...
        'Location','northeast','FontSize',9,'Box','on');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
    title(ax,sprintf('EKF SLAM  |  step %d  |  landmarks: %d / 15',t,n_lm), ...
        'FontSize',11,'FontWeight','normal');
end

%% ── Draw metrics (NO yyaxis — uses two separate axes instead) ────────────
function draw_met(ax,pe,nl,t,nt)
    % Clear and rebuild with two properly coloured lines on ONE axis
    % Normalise landmark count to same scale as error for overlay
    cla(ax); hold(ax,'on'); grid(ax,'on');
    set(ax,'FontSize',10,'Box','on');
    steps=1:t;

    % Plot pose error (left scale — actual values)
    h1=plot(ax,steps,pe,'Color',[0.15 0.42 0.90],'LineWidth',1.8);

    % Plot normalised landmark count on same axis with right ylabel via text
    nl_norm=nl*(max(pe+0.001)/15); % scale landmarks to match error range
    h2=plot(ax,steps,nl_norm,'Color',[0.85 0.10 0.40],'LineWidth',1.8,'LineStyle','--');

    % Y-ticks showing both scales
    yticks_err=linspace(0,max(pe+0.01)*1.3,5);
    set(ax,'YTick',yticks_err,'YTickLabel',arrayfun(@(v)sprintf('%.2f',v), ...
        yticks_err,'UniformOutput',false));

    % Second y-axis label via annotation (avoids yyaxis color conflict)
    ylabel(ax,'Pose error [m]  /  — — Landmarks (scaled)','FontSize',9);
    xlabel(ax,'Time step');
    title(ax,sprintf('Step %d / %d  |  LMs found: %d',t,nt,round(nl(end))), ...
        'FontSize',11,'FontWeight','normal');
    xlim(ax,[0 nt]);
    ylim(ax,[0 max(pe+0.01)*1.4]);

    legend(ax,[h1,h2],{'Pose error [m]','Landmarks found (scaled)'}, ...
        'Location','northwest','FontSize',9,'Box','on');
end

%% ── Shared EKF + sim ─────────────────────────────────────────────────────
function s=mmove(s,v,w,dt)
    th=s(3); s(1)=s(1)+v*cos(th+w*dt/2)*dt;
    s(2)=s(2)+v*sin(th+w*dt/2)*dt; s(3)=wangle(s(3)+w*dt);
end
function [mu,Sigma]=ekf_pred(mu,Sigma,v,w,dt,Qr)
    th=mu(3); n=length(mu);
    mu(1)=mu(1)+v*cos(th+w*dt/2)*dt; mu(2)=mu(2)+v*sin(th+w*dt/2)*dt;
    mu(3)=wangle(mu(3)+w*dt);
    G=eye(n); G(1,3)=-v*sin(th+w*dt/2)*dt; G(2,3)=v*cos(th+w*dt/2)*dt;
    Qf=zeros(n); Qf(1:3,1:3)=Qr; Sigma=G*Sigma*G'+Qf;
end
function [obs,rays]=do_lidar(st,lms,rng,sr,sp)
    obs=[]; rays=[];
    for i=1:size(lms,1)
        dx=lms(i,1)-st(1); dy=lms(i,2)-st(2); r=sqrt(dx^2+dy^2);
        if r>rng, continue; end
        phi=wangle(atan2(dy,dx)-st(3));
        obs=[obs;i,r+randn*sr,wangle(phi+randn*sp)]; %#ok
        rays=[rays;st(1),st(2),lms(i,1),lms(i,2)]; %#ok
    end
end
function [mu,Sigma,reg]=ekf_upd(mu,Sigma,obs,reg,R,th)
    if isempty(obs),return;end
    nL=(length(mu)-3)/2;
    for k=1:size(obs,1)
        tid=obs(k,1); ro=obs(k,2); po=obs(k,3);
        if isKey(reg,int32(tid)), idx=reg(int32(tid));
        else
            best=-1; bd=th;
            for j=0:nL-1
                mi=3+j*2+1;
                zh=zhat(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
                inn=[ro-zh(1);wangle(po-zh(2))];
                d=sqrt(inn(1)^2+inn(2)^2*4);
                if d<bd,bd=d;best=j;end
            end
            if best>=0,idx=best;reg(int32(tid))=idx;
            else
                mu=[mu;mu(1)+ro*cos(po+mu(3));mu(2)+ro*sin(po+mu(3))];
                n=length(mu); Sn=zeros(n); Sn(1:n-2,1:n-2)=Sigma;
                Sn(n-1,n-1)=1;Sn(n,n)=1;Sigma=Sn;idx=nL;nL=nL+1;
                reg(int32(tid))=idx;
            end
        end
        mi=3+idx*2+1;
        zh=zhat(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
        inn=[ro-zh(1);wangle(po-zh(2))];
        Hs=Hjac(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
        n=length(mu); H=zeros(2,n);
        H(:,1:3)=Hs(:,1:3); H(:,mi:mi+1)=Hs(:,4:5);
        S=H*Sigma*H'+R; K=Sigma*H'/S;
        mu=mu+K*inn; mu(3)=wangle(mu(3));
        Sigma=(eye(n)-K*H)*Sigma;
    end
end
function z=zhat(rx,ry,rth,mx,my)
    dx=mx-rx;dy=my-ry;z=[sqrt(dx^2+dy^2);wangle(atan2(dy,dx)-rth)];
end
function H=Hjac(rx,ry,~,mx,my)
    dx=mx-rx;dy=my-ry;r2=dx^2+dy^2;r=sqrt(r2);
    H=[-dx/r,-dy/r,0,dx/r,dy/r;dy/r2,-dx/r2,-1,-dy/r2,dx/r2];
end
function [v,w]=ctrl(st,wp,i)
    dx=wp(i,1)-st(1);dy=wp(i,2)-st(2);
    v=min(0.5,sqrt(dx^2+dy^2))*0.8; w=wangle(atan2(dy,dx)-st(3))*2;
end
function a=wangle(a),a=mod(a+pi,2*pi)-pi;end