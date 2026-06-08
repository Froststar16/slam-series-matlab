%%  Occupancy Grid Mapping from 2D LiDAR 
%
%  Fixes vs previous version:
%    1. Legend on right panel uses explicit gobject handles
%    2. imshow axis setup corrected so grid aligns with world coordinates
%    3. Wall colours consistent between panels

clear; clc; close all;
rng(42);

%% ── Parameters ───────────────────────────────────────────────────────────
DT=0.1; N_STEPS=300; WORLD_SIZE=9;
SIG_V=0.08; SIG_W=0.04; SIG_R=0.06; SIG_PHI=0.03;
Q_robot  = diag([SIG_V^2, SIG_V^2, SIG_W^2]);
R_sensor = diag([SIG_R^2, SIG_PHI^2]);
ASSOC_TH = 0.85;

N_BEAMS    = 180;
BEAM_RANGE = 4.0;
BEAM_ANGLES= linspace(-pi/2,pi/2,N_BEAMS);

GRID_RES  = 0.05;
GRID_CELLS= ceil(WORLD_SIZE/GRID_RES);
L_OCC= 0.85; L_FREE=0.40; L_MIN=-5.0; L_MAX=5.0;

true_lm=[2,2;4,1;5,3;3,5;1,4;6,2;7,4;5,6;2,7;7,7;0.5,6;4,8;8,1;8,5;1,1];
walls=[1.5,1.5,1.5,4.0; 3.0,3.0,6.0,3.0; 6.0,5.0,6.0,8.0];
waypts=[1,1;7,1;7,8;1,8;1,1;4,4;7,1;1,8;7,8;1,1];
wp_idx=1;

%% ── Init ─────────────────────────────────────────────────────────────────
ts=[0.5;0.5;pi/4]; mu=[0.5;0.5;pi/4]; Sigma=zeros(3,3);
lm_reg=containers.Map('KeyType','int32','ValueType','int32');
log_odds=zeros(GRID_CELLS,GRID_CELLS);

%% ── Figure ───────────────────────────────────────────────────────────────
fig=figure('Color','w','Position',[60 60 1100 520],'Renderer','painters');
tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
ax_og=nexttile(tl);
ax_lm=nexttile(tl); hold(ax_lm,'on'); axis(ax_lm,'equal');

fprintf('Building occupancy grid...\n');

%% ── Main loop ────────────────────────────────────────────────────────────
for t=1:N_STEPS
    [v,w]=ctrl(ts,waypts,wp_idx);
    if norm(ts(1:2)-waypts(wp_idx,:)')<0.3
        wp_idx=mod(wp_idx,size(waypts,1))+1;
    end
    ts=mmove(ts,v+randn*SIG_V,w+randn*SIG_W,DT);
    [mu,Sigma]=ekf_pred(mu,Sigma,v,w,DT,Q_robot);
    [obs_lm,~]=do_lidar_lm(ts,true_lm,BEAM_RANGE/1.2,SIG_R,SIG_PHI);
    [mu,Sigma,lm_reg]=ekf_upd(mu,Sigma,obs_lm,lm_reg,R_sensor,ASSOC_TH);

    scan=dense_scan(mu,walls,true_lm,BEAM_ANGLES,BEAM_RANGE,SIG_R);
    log_odds=update_grid(log_odds,mu,scan,BEAM_ANGLES,GRID_RES, ...
        GRID_CELLS,L_OCC,L_FREE,L_MIN,L_MAX);

    if mod(t,4)==0
        draw_og(ax_og,log_odds,mu,ts,walls,GRID_RES,GRID_CELLS,WORLD_SIZE,t);
        draw_lm(ax_lm,ts,mu,Sigma,true_lm,walls,WORLD_SIZE,t);
        drawnow limitrate;
    end
end
fprintf('Done.\n');

%% ══════════════════════════════════════════════════════════════════════════
%% DENSE LIDAR + GRID
%% ══════════════════════════════════════════════════════════════════════════
function scan=dense_scan(pose,walls,lms,angles,max_r,sr)
    N=length(angles); scan=nan(N,1);
    rx=pose(1);ry=pose(2);rth=pose(3);
    for i=1:N
        ba=rth+angles(i); bx=cos(ba); by=sin(ba); mr=max_r;
        for wi=1:size(walls,1)
            r=ray_seg(rx,ry,bx,by,walls(wi,1),walls(wi,2),walls(wi,3),walls(wi,4));
            if ~isnan(r)&&r<mr,mr=r;end
        end
        for j=1:size(lms,1)
            dx=lms(j,1)-rx;dy=lms(j,2)-ry;tc=dx*bx+dy*by;
            if tc>0,dc2=(dx-tc*bx)^2+(dy-tc*by)^2;
                if dc2<0.15^2,r=tc-sqrt(max(0,0.15^2-dc2));
                    if r>0&&r<mr,mr=r;end;end;end
        end
        if mr<max_r,scan(i)=mr+randn*sr*0.5;end
    end
end

function r=ray_seg(ox,oy,dx,dy,x1,y1,x2,y2)
    r=NaN;ex=x2-x1;ey=y2-y1;denom=dx*ey-dy*ex;
    if abs(denom)<1e-10,return;end
    t1=((x1-ox)*ey-(y1-oy)*ex)/denom;
    t2=((x1-ox)*dy-(y1-oy)*dx)/denom;
    if t1>0.01&&t2>=0&&t2<=1,r=t1;end
end

function lg=update_grid(lg,pose,scan,angles,res,cells,lo,lf,lmin,lmax)
    rx=pose(1);ry=pose(2);rth=pose(3);
    for i=1:length(angles)
        if isnan(scan(i)),continue;end
        ba=rth+angles(i);
        hx_w=rx+scan(i)*cos(ba); hy_w=ry+scan(i)*sin(ba);
        [cx0,cy0]=w2g(rx,ry,res,cells);
        [hx,hy]=w2g(hx_w,hy_w,res,cells);
        fc=bresenham(cx0,cy0,hx,hy);
        for j=1:size(fc,1)-1
            cx=fc(j,1);cy=fc(j,2);
            if cx>=1&&cx<=cells&&cy>=1&&cy<=cells
                lg(cy,cx)=max(lmin,lg(cy,cx)-lf);
            end
        end
        if hx>=1&&hx<=cells&&hy>=1&&hy<=cells
            lg(hy,hx)=min(lmax,lg(hy,hx)+lo);
        end
    end
end

function [gx,gy]=w2g(wx,wy,res,cells)
    gx=max(1,min(cells,floor(wx/res)+1));
    gy=max(1,min(cells,floor(wy/res)+1));
end

function pts=bresenham(x0,y0,x1,y1)
    dx=abs(x1-x0);dy=abs(y1-y0);
    sx=sign(x1-x0);sy=sign(y1-y0);
    err=dx-dy;pts=[];x=x0;y=y0;
    for iter=1:max(dx,dy)+2 %#ok<FXUP>
        pts=[pts;x,y];%#ok
        if x==x1&&y==y1,break;end
        e2=2*err;
        if e2>-dy,err=err-dy;x=x+sx;end
        if e2<dx,err=err+dx;y=y+sy;end
    end
end

%% ══════════════════════════════════════════════════════════════════════════
%% DRAW
%% ══════════════════════════════════════════════════════════════════════════
function draw_og(ax,lg,mu,ts,walls,res,cells,ws,t)
    prob=1-1./(1+exp(lg));
    img=repmat(1-prob,[1,1,3]);

    % Mark EKF robot as blue dot, true robot as green dot
    [rx_g,ry_g]=w2g(mu(1),mu(2),res,cells);
    [tx_g,ty_g]=w2g(ts(1),ts(2),res,cells);
    r=2; % radius in cells
    for dy=-r:r, for dx=-r:r
        if dx^2+dy^2<=r^2
            gx=rx_g+dx;gy=ry_g+dy;
            if gx>=1&&gx<=cells&&gy>=1&&gy<=cells
                img(gy,gx,:)=[0.15,0.42,0.90];
            end
            gx=tx_g+dx;gy=ty_g+dy;
            if gx>=1&&gx<=cells&&gy>=1&&gy<=cells
                img(gy,gx,:)=[0.18,0.69,0.18];
            end
        end
    end;end

    cla(ax);
    image(ax,[0 ws],[0 ws],img);
    set(ax,'YDir','normal');
    hold(ax,'on');

    % Walls
    for i=1:size(walls,1)
        plot(ax,[walls(i,1) walls(i,3)],[walls(i,2) walls(i,4)], ...
            '-','Color',[0.10 0.35 0.90],'LineWidth',2.5);
    end

    % Legend with explicit handles
    h(1)=plot(ax,NaN,NaN,'s','Color',[1 1 1],'MarkerFaceColor',[1 1 1],'MarkerSize',10);
    h(2)=plot(ax,NaN,NaN,'s','Color',[0 0 0],'MarkerFaceColor',[0.3 0.3 0.3],'MarkerSize',10);
    h(3)=plot(ax,NaN,NaN,'o','Color',[0.18 0.69 0.18],'MarkerFaceColor',[0.18 0.69 0.18],'MarkerSize',8);
    h(4)=plot(ax,NaN,NaN,'o','Color',[0.15 0.42 0.90],'MarkerFaceColor',[0.15 0.42 0.90],'MarkerSize',8);
    h(5)=plot(ax,NaN,NaN,'-','Color',[0.10 0.35 0.90],'LineWidth',2.5);
    legend(ax,h,{'Free space','Occupied','True robot','EKF estimate','Walls'}, ...
        'Location','northeast','FontSize',9,'Box','on');

    axis(ax,[0 ws 0 ws]); grid(ax,'on');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
    title(ax,sprintf('Occupancy grid  |  step %d  (%.0f cm/cell)',t,res*100), ...
        'FontSize',11,'FontWeight','normal');
end

function draw_lm(ax,ts,mu,Sigma,lms,walls,ws,t)
    cla(ax); hold(ax,'on'); axis(ax,'equal',[0 ws 0 ws]); grid(ax,'on');
    set(ax,'FontSize',10,'Box','on');

    % Walls
    h(1)=plot(ax,NaN,NaN,'-','Color',[0.10 0.35 0.90],'LineWidth',2.5);
    for i=1:size(walls,1)
        plot(ax,[walls(i,1) walls(i,3)],[walls(i,2) walls(i,4)], ...
            '-','Color',[0.10 0.35 0.90],'LineWidth',2.5,'HandleVisibility','off');
    end

    h(2)=plot(ax,NaN,NaN,'^','Color',[1.00 0.55 0.00], ...
        'MarkerFaceColor',[1.00 0.55 0.00],'MarkerSize',8);
    plot(ax,lms(:,1),lms(:,2),'^','Color',[1.00 0.55 0.00], ...
        'MarkerFaceColor',[1.00 0.55 0.00],'MarkerSize',8,'HandleVisibility','off');

    n_lm=(length(mu)-3)/2; plotted=false;
    for j=0:n_lm-1
        mi=3+j*2+1; mx=mu(mi); my=mu(mi+1);
        if ~plotted
            h(3)=plot(ax,mx,my,'p','Color',[0.85 0.10 0.40], ...
                'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',8);
            plotted=true;
        else
            plot(ax,mx,my,'p','Color',[0.85 0.10 0.40], ...
                'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',8,'HandleVisibility','off');
        end
        Slm=Sigma(mi:mi+1,mi:mi+1);
        [V,D]=eig((Slm+Slm')/2);
        ang=linspace(0,2*pi,50);
        e=3*V*sqrt(max(D,0))*[cos(ang);sin(ang)];
        plot(ax,mx+e(1,:),my+e(2,:),'--','Color',[0.55 0.55 0.55 0.45], ...
            'LineWidth',0.7,'HandleVisibility','off');
    end
    if ~plotted
        h(3)=plot(ax,NaN,NaN,'p','Color',[0.85 0.10 0.40], ...
            'MarkerFaceColor',[0.85 0.10 0.40],'MarkerSize',8);
    end

    h(4)=plot(ax,ts(1),ts(2),'o','Color',[0.18 0.69 0.18], ...
        'MarkerFaceColor',[0.18 0.69 0.18],'MarkerSize',10);
    quiver(ax,ts(1),ts(2),0.3*cos(ts(3)),0.3*sin(ts(3)),0, ...
        'Color',[0.18 0.69 0.18],'LineWidth',2,'MaxHeadSize',2,'HandleVisibility','off');
    h(5)=plot(ax,mu(1),mu(2),'o','Color',[0.15 0.42 0.90], ...
        'MarkerFaceColor',[0.15 0.42 0.90],'MarkerSize',9);
    quiver(ax,mu(1),mu(2),0.3*cos(mu(3)),0.3*sin(mu(3)),0, ...
        'Color',[0.15 0.42 0.90],'LineWidth',2,'MaxHeadSize',2,'HandleVisibility','off');
    Se=Sigma(1:2,1:2); [V,D]=eig((Se+Se')/2);
    ang=linspace(0,2*pi,50);
    e=3*V*sqrt(max(D,0))*[cos(ang);sin(ang)];
    plot(ax,mu(1)+e(1,:),mu(2)+e(2,:),'-','Color',[0.15 0.42 0.90 0.40], ...
        'LineWidth',1,'HandleVisibility','off');

    legend(ax,h,{'Walls','True LM','Est. LM (3σ)','True robot','EKF estimate'}, ...
        'Location','northeast','FontSize',9,'Box','on');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
    title(ax,sprintf('EKF landmark map  |  step %d  |  LMs: %d',t,n_lm), ...
        'FontSize',11,'FontWeight','normal');
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
function [obs,rays]=do_lidar_lm(st,lms,rng,sr,sp)
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