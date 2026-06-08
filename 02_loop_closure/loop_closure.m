%% EKF SLAM + Loop Closure Detection 

%    1. Loop closure fires at most ONCE per waypoint cycle (cooldown timer)
%    2. Stricter thresholds:  LC_LM_MATCH_TH = 0.5m,  LC_MIN_MATCHES = 4
%    3. Legend uses explicit handle array — correct colours guaranteed
%    4. Drift plot uses single-axis two-line approach (no yyaxis colour bug)
%    5. Closure arrows limited to max 10 displayed (map stays readable)

clear; clc; close all;
rng(42);

%% ── Parameters ───────────────────────────────────────────────────────────
DT=0.1; N_STEPS=400; LIDAR_RANGE=4.0; WORLD_SIZE=9;
SIG_V=0.10; SIG_W=0.05;
SIG_R=0.07; SIG_PHI=0.035;
Q_robot  = diag([SIG_V^2, SIG_V^2, SIG_W^2]);
R_sensor = diag([SIG_R^2, SIG_PHI^2]);
ASSOC_TH = 0.85;

% ── Loop closure params (tightened) ──────────────────────────────────────
LC_SUBMAP_EVERY  = 60;    % store a submap every N steps
LC_COOLDOWN      = 50;    % minimum steps between closure events
LC_LM_MATCH_TH   = 0.50; % TIGHTER: landmarks must be within 0.5 m to match
LC_MIN_MATCHES   = 4;     % STRICTER: need 4 matching landmarks
LC_SKIP_RECENT   = 3;     % skip the 3 most recent submaps (too close in time)
LC_NOISE = diag([0.04^2, 0.04^2, 0.02^2]);

true_lm=[2,2;4,1;5,3;3,5;1,4;6,2;7,4;5,6;2,7;7,7;0.5,6;4,8;8,1;8,5;1,1];
waypts=[1,1;7,1;7,8;1,8;1,1;7,1;7,8;1,8;1,1;4,4];
wp_idx=1;

%% ── Initial state ────────────────────────────────────────────────────────
ts=[0.5;0.5;pi/4]; mu=[0.5;0.5;pi/4]; Sigma=zeros(3,3);
lm_reg=containers.Map('KeyType','int32','ValueType','int32');

tp=zeros(N_STEPS+1,2); tp(1,:)=ts(1:2)';
ep=zeros(N_STEPS+1,2); ep(1,:)=mu(1:2)';
drift_log=zeros(N_STEPS,1);
closure_events=[];       % [step, xb, yb, xa, ya]
last_closure_step=-LC_COOLDOWN;  % cooldown tracker
submap_hist=struct('step',{},'lm_xy',{});

%% ── Figure ───────────────────────────────────────────────────────────────
fig=figure('Color','w','Position',[60 60 1100 500],'Renderer','painters');
tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
ax_m=nexttile(tl); hold(ax_m,'on'); axis(ax_m,'equal');
ax_d=nexttile(tl); hold(ax_d,'on');

fprintf('Running loop closure simulation...\n');

%% ── Main loop ────────────────────────────────────────────────────────────
for t=1:N_STEPS
    [v,w]=ctrl(ts,waypts,wp_idx);
    if norm(ts(1:2)-waypts(wp_idx,:)')<0.3
        wp_idx=mod(wp_idx,size(waypts,1))+1;
    end
    ts=mmove(ts,v+randn*SIG_V,w+randn*SIG_W,DT);
    [mu,Sigma]=ekf_pred(mu,Sigma,v,w,DT,Q_robot);
    [obs,rays,vis_ids]=do_lidar(ts,true_lm,LIDAR_RANGE,SIG_R,SIG_PHI);
    [mu,Sigma,lm_reg]=ekf_upd(mu,Sigma,obs,lm_reg,R_sensor,ASSOC_TH);

    %% Store submap snapshot
    if mod(t,LC_SUBMAP_EVERY)==0
        snap_xy=[];
        for id=vis_ids
            if isKey(lm_reg,int32(id))
                ei=lm_reg(int32(id)); mi=3+ei*2+1;
                snap_xy=[snap_xy;mu(mi),mu(mi+1)]; %#ok
            end
        end
        if size(snap_xy,1)>=LC_MIN_MATCHES
            submap_hist(end+1)=struct('step',t,'lm_xy',snap_xy); %#ok
        end
    end

    %% Loop closure check — with cooldown guard
    can_close = (t > LC_SUBMAP_EVERY*(LC_SKIP_RECENT+1)) && ...
                (t - last_closure_step >= LC_COOLDOWN) && ...
                (length(submap_hist) > LC_SKIP_RECENT);

    if can_close
        cur_xy=[];
        for id=vis_ids
            if isKey(lm_reg,int32(id))
                ei=lm_reg(int32(id)); mi=3+ei*2+1;
                cur_xy=[cur_xy;mu(mi),mu(mi+1)]; %#ok
            end
        end

        if size(cur_xy,1)>=LC_MIN_MATCHES
            [closed,corr]=check_lc(cur_xy,submap_hist, ...
                LC_SKIP_RECENT,LC_LM_MATCH_TH,LC_MIN_MATCHES);
            if closed
                pb=mu(1:2)';
                [mu,Sigma]=apply_lc(mu,Sigma,corr,LC_NOISE);
                closure_events=[closure_events;t,pb(1),pb(2),mu(1),mu(2)]; %#ok
                last_closure_step=t;
                fprintf('  Loop closure @ step %d  (dx=%.3f m, dy=%.3f m)\n', ...
                    t,corr(1),corr(2));
            end
        end
    end

    tp(t+1,:)=ts(1:2)'; ep(t+1,:)=mu(1:2)';
    drift_log(t)=norm(ts(1:2)-mu(1:2));

    if mod(t,4)==0
        draw_lc(ax_m,ax_d,ts,mu,Sigma, ...
            tp(1:t+1,:),ep(1:t+1,:),true_lm,rays, ...
            closure_events,drift_log(1:t),t,N_STEPS,WORLD_SIZE);
        drawnow limitrate;
    end
end

n_closed=size(closure_events,1);
fprintf('Done. Loop closures: %d\n',n_closed);

%% ══════════════════════════════════════════════════════════════════════════
%% LOOP CLOSURE LOGIC
%% ══════════════════════════════════════════════════════════════════════════
function [closed,corr]=check_lc(cur_xy,hist,skip_recent,match_th,min_match)
    closed=false; corr=zeros(3,1);
    % Only compare against older submaps (skip the most recent N)
    n=length(hist);
    for i=1:n-skip_recent
        s=hist(i);
        if size(s.lm_xy,1)<min_match, continue; end
        n_match=0; offset=zeros(1,2);
        for c=1:size(cur_xy,1)
            dists=sqrt(sum((s.lm_xy-cur_xy(c,:)).^2,2));
            [md,mi]=min(dists);
            if md<match_th
                n_match=n_match+1;
                offset=offset+(s.lm_xy(mi,:)-cur_xy(c,:));
            end
        end
        if n_match>=min_match
            offset=offset/n_match;
            corr=[offset(1);offset(2);0];
            closed=true; return;
        end
    end
end

function [mu,Sigma]=apply_lc(mu,Sigma,corr,Q_lc)
    n=length(mu);
    H=zeros(3,n); H(1,1)=1; H(2,2)=1; H(3,3)=1;
    innov=corr; innov(3)=wangle(innov(3));
    S=H*Sigma*H'+Q_lc; K=Sigma*H'/S;
    mu=mu+K*innov; mu(3)=wangle(mu(3));
    Sigma=(eye(n)-K*H)*Sigma;
end

%% ══════════════════════════════════════════════════════════════════════════
%% DRAW — explicit handle array, no yyaxis
%% ══════════════════════════════════════════════════════════════════════════
function draw_lc(axm,axd,ts,mu,Sigma,tp,ep,lms,rays,ce,drift,t,nt,ws)

    %% ── Map panel ────────────────────────────────────────────────────────
    cla(axm); hold(axm,'on');
    axis(axm,'equal',[0 ws 0 ws]); grid(axm,'on');
    set(axm,'FontSize',10,'Box','on');

    % LiDAR rays (no legend entry)
    for i=1:size(rays,1)
        line(axm,[rays(i,1) rays(i,3)],[rays(i,2) rays(i,4)], ...
            'Color',[0 0.74 0.84 0.25],'LineWidth',0.7,'HandleVisibility','off');
    end

    % Named plot handles for legend
    h(1)=plot(axm,NaN,NaN,'-','Color',[0 0.74 0.84],'LineWidth',1.5); % LiDAR dummy
    h(2)=plot(axm,tp(:,1),tp(:,2),'-','Color',[0.18 0.69 0.18],'LineWidth',1.5);
    h(3)=plot(axm,ep(:,1),ep(:,2),'-','Color',[0.15 0.42 0.90],'LineWidth',1.5);
    h(4)=plot(axm,lms(:,1),lms(:,2),'^','Color',[1.00 0.55 0.00], ...
        'MarkerFaceColor',[1.00 0.55 0.00],'MarkerSize',9,'HandleVisibility','off');

    % True landmark dummy for legend
    h(4)=plot(axm,NaN,NaN,'^','Color',[1.00 0.55 0.00], ...
        'MarkerFaceColor',[1.00 0.55 0.00],'MarkerSize',9);
    plot(axm,lms(:,1),lms(:,2),'^','Color',[1.00 0.55 0.00], ...
        'MarkerFaceColor',[1.00 0.55 0.00],'MarkerSize',9,'HandleVisibility','off');

    % EKF landmarks + covariance ellipses
    n_lm=(length(mu)-3)/2; plotted_lm=false;
    for j=0:n_lm-1
        mi=3+j*2+1; mx=mu(mi); my=mu(mi+1);
        if ~plotted_lm
            h(5)=plot(axm,mx,my,'p','Color',[0.85 0.10 0.40], ...
                'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',8);
            plotted_lm=true;
        else
            plot(axm,mx,my,'p','Color',[0.85 0.10 0.40], ...
                'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',8, ...
                'HandleVisibility','off');
        end
        Slm=Sigma(mi:mi+1,mi:mi+1);
        [V,D]=eig((Slm+Slm')/2);
        ang=linspace(0,2*pi,50);
        e=3*V*sqrt(max(D,0))*[cos(ang);sin(ang)];
        plot(axm,mx+e(1,:),my+e(2,:),'--', ...
            'Color',[0.55 0.55 0.55 0.45],'LineWidth',0.7,'HandleVisibility','off');
    end
    if ~plotted_lm
        h(5)=plot(axm,NaN,NaN,'p','Color',[0.85 0.10 0.40], ...
            'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',8);
    end

    % Loop closure arrows (show max 8 to keep map readable)
    plotted_ce=false;
    n_show=min(size(ce,1),8);
    idx_show=round(linspace(1,size(ce,1),n_show));
    for i=idx_show
        if ~plotted_ce
            h(6)=plot(axm,[ce(i,2) ce(i,4)],[ce(i,3) ce(i,5)],'r--', ...
                'LineWidth',1.4);
            plot(axm,ce(i,4),ce(i,5),'rs','MarkerFaceColor','r', ...
                'MarkerSize',6,'HandleVisibility','off');
            plotted_ce=true;
        else
            plot(axm,[ce(i,2) ce(i,4)],[ce(i,3) ce(i,5)],'r--', ...
                'LineWidth',1.4,'HandleVisibility','off');
            plot(axm,ce(i,4),ce(i,5),'rs','MarkerFaceColor','r', ...
                'MarkerSize',6,'HandleVisibility','off');
        end
    end
    if ~plotted_ce
        h(6)=plot(axm,NaN,NaN,'r--','LineWidth',1.4); % empty dummy
    end

    % Robot positions
    plot(axm,ts(1),ts(2),'o','Color',[0.18 0.69 0.18], ...
        'MarkerFaceColor',[0.18 0.69 0.18],'MarkerSize',10,'HandleVisibility','off');
    quiver(axm,ts(1),ts(2),0.3*cos(ts(3)),0.3*sin(ts(3)),0, ...
        'Color',[0.18 0.69 0.18],'LineWidth',2,'MaxHeadSize',2,'HandleVisibility','off');
    plot(axm,mu(1),mu(2),'o','Color',[0.15 0.42 0.90], ...
        'MarkerFaceColor',[0.15 0.42 0.90],'MarkerSize',9,'HandleVisibility','off');
    quiver(axm,mu(1),mu(2),0.3*cos(mu(3)),0.3*sin(mu(3)),0, ...
        'Color',[0.15 0.42 0.90],'LineWidth',2,'MaxHeadSize',2,'HandleVisibility','off');
    Se=Sigma(1:2,1:2); [V,D]=eig((Se+Se')/2);
    ang=linspace(0,2*pi,50);
    e=3*V*sqrt(max(D,0))*[cos(ang);sin(ang)];
    plot(axm,mu(1)+e(1,:),mu(2)+e(2,:),'-', ...
        'Color',[0.15 0.42 0.90 0.40],'LineWidth',1,'HandleVisibility','off');

    legend(axm,h,{'LiDAR ray','True path','EKF path','True LM', ...
        'Est. LM','Loop closure'},'Location','northeast','FontSize',9,'Box','on');
    xlabel(axm,'x [m]'); ylabel(axm,'y [m]');
    title(axm,sprintf('Loop Closure SLAM  |  step %d  |  closures: %d', ...
        t,size(ce,1)),'FontSize',11,'FontWeight','normal');

    %% ── Drift panel — two lines, one axis, explicit colours ──────────────
    cla(axd); hold(axd,'on'); grid(axd,'on');
    set(axd,'FontSize',10,'Box','on');
    steps=1:t;

    hd1=plot(axd,steps,drift,'Color',[0.15 0.42 0.90],'LineWidth',1.8);

    % Vertical markers at closure events (xline equivalent without xline)
    for i=1:size(ce,1)
        line(axd,[ce(i,1) ce(i,1)],[0 max(drift)*1.5+0.01], ...
            'Color',[0.85 0.10 0.10 0.50],'LineStyle','--', ...
            'LineWidth',1.0,'HandleVisibility','off');
    end
    hd2=plot(axd,NaN,NaN,'--','Color',[0.85 0.10 0.10],'LineWidth',1.4);

    legend(axd,[hd1,hd2],{'Pose error [m]','Loop closure event'}, ...
        'Location','northwest','FontSize',9,'Box','on');
    xlabel(axd,'Time step');
    ylabel(axd,'Pose error [m]');
    title(axd,sprintf('Drift  |  closures: %d',size(ce,1)), ...
        'FontSize',11,'FontWeight','normal');
    xlim(axd,[0 nt]);
    ylim(axd,[0 max(drift(1:t))*1.4+0.01]);
end

%% ── Shared helpers ───────────────────────────────────────────────────────
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
function [obs,rays,vis]=do_lidar(st,lms,rng,sr,sp)
    obs=[]; rays=[]; vis=[];
    for i=1:size(lms,1)
        dx=lms(i,1)-st(1); dy=lms(i,2)-st(2); r=sqrt(dx^2+dy^2);
        if r>rng,continue;end
        phi=wangle(atan2(dy,dx)-st(3));
        obs=[obs;i,r+randn*sr,wangle(phi+randn*sp)]; %#ok
        rays=[rays;st(1),st(2),lms(i,1),lms(i,2)]; %#ok
        vis=[vis,i]; %#ok
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
                mi=3+j*2+1;
                zh=zhat(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
                inn=[ro-zh(1);wangle(po-zh(2))];
                d=sqrt(inn(1)^2+inn(2)^2*4);
                if d<bd,bd=d;best=j;end
            end
            if best>=0,idx=best;reg(int32(tid))=idx;
            else
                mu=[mu;mu(1)+ro*cos(po+mu(3));mu(2)+ro*sin(po+mu(3))];
                n=length(mu);Sn=zeros(n);Sn(1:n-2,1:n-2)=Sigma;
                Sn(n-1,n-1)=1;Sn(n,n)=1;Sigma=Sn;idx=nL;nL=nL+1;
                reg(int32(tid))=idx;
            end
        end
        mi=3+idx*2+1;
        zh=zhat(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
        inn=[ro-zh(1);wangle(po-zh(2))];
        Hs=Hjac(mu(1),mu(2),mu(3),mu(mi),mu(mi+1));
        n=length(mu);H=zeros(2,n);
        H(:,1:3)=Hs(:,1:3);H(:,mi:mi+1)=Hs(:,4:5);
        S=H*Sigma*H'+R;K=Sigma*H'/S;
        mu=mu+K*inn;mu(3)=wangle(mu(3));
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