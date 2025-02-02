%% Process and show results
function visualize_rescaling(x, t, N, na, moving, videofile, form, az, el, i1)

    global n1 w Tsim v

    P    = reshape(x(:,      1: 3*na),[],3,1,na);
    V    = reshape(x(:, 3*na+1: 6*na),[],3,1,na);
    R    = reshape(x(:, 6*na+1:15*na),[],3,3,na);
    Pdes = reshape(x(:,15*na+1:18*na),[],3,1,na);
   
    for it = 1:length(t)
        
        if it<i1
            R3ds = get_rd3(P(it,:,:,:), V(it,:,:,:), t(it), 1);
        else
            R3ds = get_rd3(P(it,:,:,:), V(it,:,:,:), t(it), 2);
        end
        
        for i = 1:na
            if it == 1
                Vdes = [0 0 0];
            else
                Vdes = (Pdes(it,:,:,i)-Pdes(it-1,:,:,i))./(t(it)-t(it-1));
            end
            error_P(it,i) = norm(reshape(P(it,:,:,i) - Pdes(it,:,:,i),3,1));
            error_V(it,i) = norm(reshape(V(it,:,:,i) - Vdes,3,1));
            error_R(it,i) = norm(reshape(R(it,:,3,i)' - R3ds(:,i),3,1));
        end
    end
    
    iend = length(t);
    
    f1 = figure;
    set(f1,'Renderer','painters');
    colors = colormap(lines);
    colors(1:4,:) = colors([4,1,2,3],:);
%     subplot(4,3,10)
    subplot(3,1,1)
    plot(t, error_P(:,2), 'Color', colors(2,:), 'LineWidth', 1.5)
    hold on
    plot(t, error_P(:,3), 'Color', colors(3,:), 'LineWidth', 1.5)
    plot(t, error_P(:,4), 'Color', colors(4,:), 'LineWidth', 1.5)
    ax = gca;
    plot([t(i1), t(i1)], ax.YLim, 'k--')
    xlim([0 max(t)])
    ylabel('$||\tilde{p}_i||$ $[m]$', 'Interpreter', 'latex', 'FontSize', 12)
    title('Absolute position error', 'Interpreter', 'latex')
    l=legend('Agent 2','Agent 3','Agent 4', 'Interpreter', 'latex');
    l.Position = [0.7365, 0.7812, 0.2059, 0.1684];
    grid on
    reduce_grid([2,2]);
   
%     subplot(4,3,11)
    subplot(3,1,2)
    plot(t,error_V(:,2), 'Color', colors(2,:), 'LineWidth', 1.5)
    hold on
    plot(t,error_V(:,3), 'Color', colors(3,:), 'LineWidth', 1.5)
    plot(t,error_V(:,4), 'Color', colors(4,:), 'LineWidth', 1.5)
    ax = gca;
    plot([t(i1), t(i1)], ax.YLim, 'k--')
    xlim([0 max(t)])
    ylabel('$||\tilde{v}_i||$ $[m/s]$', 'Interpreter', 'latex', 'FontSize', 12)
    title('Absolute velocity error', 'Interpreter', 'latex')
    grid on
    reduce_grid([2,2]);
    
%     subplot(4,3,12)
    subplot(3,1,3)
    plot(t,error_R(:,2), 'Color', colors(2,:), 'LineWidth', 1.5)
    hold on
    plot(t,error_R(:,3), 'Color', colors(3,:), 'LineWidth', 1.5)
    plot(t,error_R(:,4), 'Color', colors(4,:), 'LineWidth', 1.5)
    ylim([0 max(max(error_R))/3])
    ax = gca;
    plot([t(i1), t(i1)], ax.YLim, 'k--')
    xlim([0 max(t)])
    xlabel('$t$ $[s]$', 'Interpreter', 'latex')
    ylabel('$||r_{3,i} - r_{3d,i}||$ $[-]$', 'Interpreter', 'latex', 'FontSize', 12)
    title('Rotation error', 'Interpreter', 'latex')
    grid on
    reduce_grid([2,2]);

    f1.Position = [805 300 560 429];
    set(findall(f1,'-property','FontSize'),'FontSize',14); % change font size
    
    %%
    f2 = figure;
    set(f2,'Renderer','painters');
    subplot(3,1,1)
    plot(t(1:i1), error_P(1:i1,2), 'Color', colors(2,:), 'LineWidth', 1.5)
    hold on
    plot(t(1:i1), error_P(1:i1,3), 'Color', colors(3,:), 'LineWidth', 1.5)
    plot(t(1:i1), error_P(1:i1,4), 'Color', colors(4,:), 'LineWidth', 1.5)
    xlim([0 t(i1)])
    ylabel('$||\tilde{p}_i||$ $[m]$', 'Interpreter', 'latex', 'FontSize', 12)
    title('Absolute position error', 'Interpreter', 'latex')
    l=legend('Agent 2','Agent 3','Agent 4', 'Interpreter', 'latex');
    l.Position = [0.7365, 0.7812, 0.2059, 0.1684];
    grid on
    reduce_grid([2,2]);
   
%     subplot(4,3,11)
    subplot(3,1,2)
    plot(t(1:i1),error_V(1:i1,2), 'Color', colors(2,:), 'LineWidth', 1.5)
    hold on
    plot(t(1:i1),error_V(1:i1,3), 'Color', colors(3,:), 'LineWidth', 1.5)
    plot(t(1:i1),error_V(1:i1,4), 'Color', colors(4,:), 'LineWidth', 1.5)
    xlim([0 t(i1)])
    ylabel('$||\tilde{v}_i||$ $[m/s]$', 'Interpreter', 'latex', 'FontSize', 12)
    title('Absolute velocity error', 'Interpreter', 'latex')
%     legend('Agent 1','Agent 2','Agent 3','Agent 4', 'Interpreter', 'latex')
    grid on
    reduce_grid([2,2]);
    
%     subplot(4,3,12)
    subplot(3,1,3)
    plot(t(1:i1),error_R(1:i1,2), 'Color', colors(2,:), 'LineWidth', 1.5)
    hold on
    plot(t(1:i1),error_R(1:i1,3), 'Color', colors(3,:), 'LineWidth', 1.5)
    plot(t(1:i1),error_R(1:i1,4), 'Color', colors(4,:), 'LineWidth', 1.5)
    ylim([0 max(max(error_R))/3])
    xlim([0 t(i1)])
    xlabel('$t$ $[s]$', 'Interpreter', 'latex')
    ylabel('$||r_{3,i} - r_{3d,i}||$ $[-]$', 'Interpreter', 'latex', 'FontSize', 12)
    title('Rotation error', 'Interpreter', 'latex')
    grid on
    reduce_grid([2,2]);

    f2.Position = [805 300 560 429];
    set(findall(f2,'-property','FontSize'),'FontSize',14); % change font size
    
    %%
    f = figure;
    set(f,'Renderer','painters');
%     subplot(4,3,[1,2,3,4,5,6,7,8,9])
    a = 0.5;
    MS = 8;
    
    plot3(P(iend,1,1,1), P(iend,2,1,1), P(iend,3,1,1), 'd', 'Color', colors(1,:), 'MarkerFaceColor', colors(1,:), 'MarkerSize', MS)
    hold on
    plot3(P(iend,1,1,2), P(iend,2,1,2), P(iend,3,1,2),...
          'o', 'MarkerEdgeColor', colors(2,:),...
          'MarkerFaceColor', colors(2,:),'MarkerSize',MS)
    plot3(P(iend,1,1,2), P(iend,2,1,2), P(iend,3,1,2),...
          'o', 'MarkerEdgeColor', colors(2,:),...
          'MarkerFaceColor', colors(2,:),'MarkerSize',MS)
    plot3(P(1,1,1,2), P(1,2,1,2), P(1,3,1,2),...
          'o', 'MarkerEdgeColor', colors(2,:),'MarkerSize',MS)
    for i = 1:na
        if i==1 || i==4
            dstep = 20;
            plot3(Pdes(1:dstep:iend,1,1,i), Pdes(1:dstep:iend,2,1,i), Pdes(1:dstep:iend,3,1,i), '--', 'Color', colors(i,:), 'LineWidth',0.3)
            plot3(P(1:iend,1,1,i),   P(1:iend,2,1,i),   P(1:iend,3,1,i),...
              '-', 'Color', [colors(i,:),1], 'LineWidth',1)
        end
        text(P(iend,1,i)-.2, P(iend,2,i), P(iend,3,i), ['~', num2str(i)],...
             'Interpreter', 'latex', 'FontSize', 16)
        for j = N{i} % draw lines between linked agents
            p1 = P(i1,:,1,i);
            p2 = P(i1,:,1,j);
            dp = (p2 - p1);
            quiver3(p1(1),p1(2),p1(3),dp(1),dp(2),dp(3), 0, 'Color', [0.3, 0.3, 0.3], 'LineWidth', 2, 'MaxHeadSize', 3/norm(dp))
        end
        for j = N{i} % draw lines between linked agents
            p1 = P(iend,:,1,i);
            p2 = P(iend,:,1,j);
            dp = (p2 - p1);
            quiver3(p1(1),p1(2),p1(3),dp(1),dp(2),dp(3), 0, 'Color', [0.3, 0.3, 0.3], 'LineWidth', 2, 'MaxHeadSize', 3/norm(dp))
        end
        for j = N{i} % draw lines between linked agents
            p1 = P(1,:,1,i);
            p2 = P(1,:,1,j);
            dp = (p2 - p1);
            quiver3(p1(1),p1(2),p1(3),dp(1),dp(2),dp(3), 0, 'Color', [0.3, 0.3, 0.3], 'LineWidth', 2, 'MaxHeadSize', 3/norm(dp))
        end
        for k = 1:3 % draw axes
            if i~=1
                plot3([P(1,1,1,i) P(1,1,1,i)+R(1,1,k,i)*a],...
                      [P(1,2,1,i) P(1,2,1,i)+R(1,2,k,i)*a],...
                      [P(1,3,1,i) P(1,3,1,i)+R(1,3,k,i)*a],...
                      'Color', colors(i,:))
            end
            plot3([P(i1,1,1,i) P(i1,1,1,i)+R(i1,1,k,i)*a],...
                  [P(i1,2,1,i) P(i1,2,1,i)+R(i1,2,k,i)*a],...
                  [P(i1,3,1,i) P(i1,3,1,i)+R(i1,3,k,i)*a],...
                  'Color', colors(i,:))
            plot3([P(iend,1,1,i) P(iend,1,1,i)+R(iend,1,k,i)*a],...
                  [P(iend,2,1,i) P(iend,2,1,i)+R(iend,2,k,i)*a],...
                  [P(iend,3,1,i) P(iend,3,1,i)+R(iend,3,k,i)*a],...
                  'Color', colors(i,:))
        end
        if i ~= 1
            plot3(P(1,1,1,i),   P(1,2,1,i),   P(1,3,1,i),...
              'o', 'MarkerEdgeColor', colors(i,:),'MarkerSize',MS);
            plot3(P(i1,1,1,i), P(i1,2,1,i), P(i1,3,1,i),...
                  'o', 'MarkerEdgeColor', colors(i,:),...
                  'MarkerFaceColor', colors(i,:),'MarkerSize',MS);
            plot3(P(iend,1,1,i), P(iend,2,1,i), P(iend,3,1,i),...
                  'o', 'MarkerEdgeColor', colors(i,:),...
                  'MarkerFaceColor', colors(i,:),'MarkerSize',MS);
        else
            plot3(P(1,1,1,i),   P(1,2,1,i),   P(1,3,1,i),...
              'd', 'MarkerEdgeColor', colors(i,:),'MarkerSize',MS);
        end
    end
    plot3(P(i1,1,1,1), P(i1,2,1,1), P(i1,3,1,1), 'd', 'Color', colors(1,:), 'MarkerFaceColor', colors(1,:), 'MarkerSize', MS)
%     plot3(P(1,1,1,1), P(1,2,1,1), P(1,3,1,1), 'd', 'Color', colors(1,:), 'MarkerFaceColor', colors(1,:), 'MarkerSize', MS)

    d1 = 3;
    d2 = 1.5;
    x = [-d1 -d1 d1 d1 -d1 -d2 d2 d2 -d2 -d2 -d1];
    z = [-d1 d1 d1 -d1 -d1 -d2 -d2 d2 d2 -d2 -d1];
    y = Tsim/2*v*ones(1,length(x));
    fill3(x,y,z,'r','FaceColor',[.7,.7,.7],'LineStyle','none')

    f.CurrentAxes.YDir = 'Reverse';
    f.CurrentAxes.ZDir = 'Reverse';
    f.Position = [704 292 792 452];
    ax = gca;
    xlabel('$x$ $[m]$', 'Interpreter', 'latex')
    ylabel('$y$ $[m]$', 'Interpreter', 'latex')
    zlabel('$z$ $[m]$', 'Interpreter', 'latex')
    head = title('Quadrotors in a formation with rigid shape and changing scaling', 'Interpreter', 'latex','FontSize',16);
    head.Interpreter = 'latex';
    axis equal
    xlim(ax.XLim + [-.2 .2]);
    zlim(ax.ZLim + [-.2 .2]);
%     a.YLim = a.YLim + [-.5 .5];
    view(az, el)
    leg = legend('Leader position','','','Initial positions','Location','northwest','Interpreter', 'latex');
    leg.FontSize = 13;
    grid on; hold on    
    set(gca, 'Children', flipud(get(gca, 'Children')) )
    set(findall(f,'-property','FontSize'),'FontSize',14); % change font size
    
    %% Video
    if moving
        framerate = 60;
        MS = 7;
        
        if ~isempty(videofile)
            writerObj = VideoWriter(videofile);
            writerObj.FrameRate = framerate;
            open(writerObj);
        end

        f = figure;
        f.Position = [603 472 926 506];
        f.Color = 'w';
        n = get(f,'Number');

        subplot(2,3,1)
        plot(t,error_P,':')
        hold on, grid on
        h2 = plot(t,error_P);
        h3 = plot(0,[0 0 0 0],'.','MarkerSize',20);
        xlabel('$t$ $[s]$', 'Interpreter', 'latex')
        ylabel('$||\tilde{p}_i||$ $[m]$', 'Interpreter', 'latex', 'FontSize', 12)
        title('Absolute position error', 'Interpreter', 'latex')
        legend('','','','','Agent 1','Agent 2','Agent 3','Agent 4', 'Interpreter', 'latex')

        subplot(2,3,4)
        plot(t,error_V,':');
        hold on, grid on
        h4 = plot(t,error_V);
        h5 = plot(0,[0 0 0 0],'.','MarkerSize',20);
        xlabel('$t$ $[s]$', 'Interpreter', 'latex')
        ylabel('$||\tilde{v}_i||$ $[m/s]$', 'Interpreter', 'latex', 'FontSize', 12)
        title('Absolute velocity error', 'Interpreter', 'latex')
        
        for i=1:na
            h2(i).Color = colors(i,:);
            h3(i).Color = colors(i,:);
            h4(i).Color = colors(i,:);
            h5(i).Color = colors(i,:);
        end
        
        subplot(2,3,[2,3,5,6])
        ax = gca;

        h(1,1) = plot3(0, 0, 0, 'd', 'Color', colors(1,:), 'MarkerFaceColor', colors(1,:), 'MarkerSize', MS);
        hold on
%         if length(P(1,1,1,:)) > 1
%             plot3(P(1,1,1,2), P(1,2,1,2), P(1,3,1,2),...
%                   'o', 'MarkerEdgeColor', colors(2,:), 'MarkerSize', MS);
%             h(2,1) = plot3(P(1,1,1,2), P(1,2,1,2), P(1,3,1,2),...
%                            'o', 'MarkerEdgeColor', colors(2,:),...
%                            'MarkerFaceColor', colors(2,:), 'MarkerSize', MS);
%         end

        for i = 1:na
            if i >= 2
                h(i,2) = plot3(Pdes(1,1,1,i), Pdes(1,2,1,i), Pdes(1,3,1,i),...
                      'o', 'MarkerEdgeColor', colors(i,:), 'MarkerSize', MS);
                h(i,1) = plot3(0, 0, 0, 'o', 'MarkerEdgeColor', colors(i,:),...
                               'MarkerFaceColor', colors(i,:), 'MarkerSize', MS);
            end
%             for k = 1:3 % draw axes
%                 plot3([P(1,1,1,i) P(1,1,1,i)+R(1,1,k,i)*a],...
%                       [P(1,2,1,i) P(1,2,1,i)+R(1,2,k,i)*a],...
%                       [P(1,3,1,i) P(1,3,1,i)+R(1,3,k,i)*a],...
%                       'Color', colors(i,:))
%             end
%             h(i,2) = plot3(0, 0, 0, '--', 'Color', colors(i,:));
            h(i,3) = plot3(0, 0, 0, 'Color', colors(i,:));
            h(i,4) = text(P(end,1,i), P(end,2,i), P(end,3,i),...
                          ['~', num2str(i)], 'Interpreter', 'latex',...
                          'FontSize', 12);
            for j = 5:(length(N{i})+4)
                h(i,j) = quiver3(0,0,0,0,0,0,0, 'Color', [0.3, 0.3, 0.3], 'LineWidth', 2);
            end
            for j = (length(N{i})+5):(length(N{i})+7)
                h(i,j) = plot3(0,0,0,'Color', colors(i,:));
            end
            plot3(Pdes(1:iend,1,1,i), Pdes(1:iend,2,1,i), Pdes(1:iend,3,1,i), ':', 'Color', colors(i,:))
        end

        d1 = 3;
        d2 = 1.5;
        x = [-d1 -d1 d1 d1 -d1 -d2 d2 d2 -d2 -d2 -d1];
        z = [-d1 d1 d1 -d1 -d1 -d2 -d2 d2 d2 -d2 -d1];
        y = Tsim/2*v*ones(1,length(x));
        fill3(x,y,z,'r','FaceColor',[.7,.7,.7],'LineStyle','none')

        grid on
        axis equal
        view(-161.1976, 13.5689)
        xlim([min(P(:,1,1,:),[],'all')-a,max(P(:,1,1,:),[],'all')+a])
        ylim([min(P(:,2,1,:),[],'all')-a,max(P(:,2,1,:),[],'all')+a])
        zlim([min(P(:,3,1,:),[],'all')-a,max(P(:,3,1,:),[],'all')+a])
        xlabel('$x[m]$', 'Interpreter', 'latex')
        ylabel('$y[m]$', 'Interpreter', 'latex')
        zlabel('$z[m]$', 'Interpreter', 'latex')
        title({'Trajectory of the agents in the controlled formation',''}, 'Interpreter', 'latex')
        f.CurrentAxes.YDir = 'Reverse';
        f.CurrentAxes.ZDir = 'Reverse';
        nsteps = 1600;
        tstep = t(end)/nsteps;
        tprev = t(1);
        legend('Position of leader','','','','','','Desired positions of followers','Current position of followers','Location','southeast', 'Interpreter', 'latex')

        it = 1;
        
        if ~isempty(videofile)
            pause(15)
        end
        
        while it < length(t)

            while t(it) < tstep + tprev
                it = it + 1;
                if it == length(t)
                    break
                end
            end
            
            tprev = tprev + tstep;

            for i = 1:na
                h(i,1).XData = P(it,1,1,i);
                h(i,1).YData = P(it,2,1,i);
                h(i,1).ZData = P(it,3,1,i);
                if i >= 2
                    h(i,2).XData = Pdes(it,1,1,i);
                    h(i,2).YData = Pdes(it,2,1,i);
                    h(i,2).ZData = Pdes(it,3,1,i);
                end
                h(i,3).XData = P(max(1,it-10):it,1,1,i);
                h(i,3).YData = P(max(1,it-10):it,2,1,i);
                h(i,3).ZData = P(max(1,it-10):it,3,1,i);
                h(i,4).Position = [P(it,1,i)-0.1, P(it,2,i), P(it,3,i)-0.1];
                ii = 5;
                for j = N{i} % draw arrows
                    p1 = P(it,:,1,i);
                    p2 = P(it,:,1,j);
                    dp = (p2 - p1);
                    ndp = norm(dp);
                    h(i,ii).XData = p1(1);
                    h(i,ii).YData = p1(2);
                    h(i,ii).ZData = p1(3);
                    h(i,ii).UData = dp(1)*(ndp-0.1)/ndp;
                    h(i,ii).VData = dp(2)*(ndp-0.1)/ndp;
                    h(i,ii).WData = dp(3)*(ndp-0.1)/ndp;
                    h(i,ii).MaxHeadSize = 3/ndp;
                    ii = ii + 1;
                end
        
                for k = 1:3 % draw axes
                    h(i,ii).XData = [P(it,1,1,i) P(it,1,1,i)+R(it,1,k,i)*a];
                    h(i,ii).YData = [P(it,2,1,i) P(it,2,1,i)+R(it,2,k,i)*a];
                    h(i,ii).ZData = [P(it,3,1,i) P(it,3,1,i)+R(it,3,k,i)*a];
                    ii = ii + 1;
                end
                    
                h2(i).XData = t(1:it);
                h2(i).YData = error_P(1:it,i);
                h3(i).XData = t(it);
                h3(i).YData = error_P(it,i);
                h4(i).XData = t(1:it);
                h4(i).YData = error_V(1:it,i);
                h5(i).XData = t(it);
                h5(i).YData = error_V(it,i);
            
            end
            
            h(1,1).XData = P(it,1,1,1);
            h(1,1).YData = P(it,2,1,1);
            h(1,1).ZData = P(it,3,1,1);

            ax.Title.String{2} = sprintf('t = %.0f s', t(it));
            pause(1/framerate)
            if ~ishghandle(n)
                break
            end
            if ~isempty(videofile)
                writeVideo(writerObj, getframe(f));
            end
        end

        if ~isempty(videofile)
            close(writerObj);
        end
    end
    
end

function Rde3 = get_rd3(P, V, t, Q)

    global w N na kd kp m g e3 v Tsim
    
    if Q == 1
        % Define desired motion
        Pd(:,:,1) = [                           0; v*t;                            0];
        Pd(:,:,2) = [       (3-4/Tsim*t)*sin(w*t); v*t;        (3-4/Tsim*t)*cos(w*t)];
        Pd(:,:,3) = [(3-4/Tsim*t)*sin(w*t-2*pi/3); v*t; (3-4/Tsim*t)*cos(w*t-2*pi/3)];
        Pd(:,:,4) = [(3-4/Tsim*t)*sin(w*t-4*pi/3); v*t; (3-4/Tsim*t)*cos(w*t-4*pi/3)];

        Vd(:,:,1) = [                                                    0; v;                                                      0];
        Vd(:,:,2) = [              w*(3-4/Tsim*t)*cos(w*t)-4/Tsim*sin(w*t); v;               -w*(3-4/Tsim*t)*sin(w*t)-4/Tsim*cos(w*t)];
        Vd(:,:,3) = [w*(3-4/Tsim*t)*cos(w*t-2*pi/3)-4/Tsim*sin(w*t-2*pi/3); v; -w*(3-4/Tsim*t)*sin(w*t-2*pi/3)-4/Tsim*cos(w*t-2*pi/3)];
        Vd(:,:,4) = [w*(3-4/Tsim*t)*cos(w*t-4*pi/3)-4/Tsim*sin(w*t-4*pi/3); v; -w*(3-4/Tsim*t)*sin(w*t-4*pi/3)-4/Tsim*cos(w*t-4*pi/3)];

        Ud(:,:,1) = [                                                         0; 0;                                                         0];
        Ud(:,:,2) = [              -w^2*(3-4/Tsim*t)*sin(w*t)-w*8/Tsim*cos(w*t); 0;               -w^2*(3-4/Tsim*t)*cos(w*t)+w*8/Tsim*sin(w*t)];
        Ud(:,:,3) = [-w^2*(3-4/Tsim*t)*sin(w*t-2*pi/3)-w*8/Tsim*cos(w*t-2*pi/3); 0; -w^2*(3-4/Tsim*t)*cos(w*t-2*pi/3)+w*8/Tsim*sin(w*t-2*pi/3)];
        Ud(:,:,4) = [-w^2*(3-4/Tsim*t)*sin(w*t-4*pi/3)-w*8/Tsim*cos(w*t-4*pi/3); 0; -w^2*(3-4/Tsim*t)*cos(w*t-4*pi/3)+w*8/Tsim*sin(w*t-4*pi/3)];

        Uddot(:,:,1) = [                                                            0; 0;                                                             0];
        Uddot(:,:,2) = [              -w^3*(3-4/Tsim*t)*cos(w*t)+w^2*12/Tsim*sin(w*t); 0;               -w^3*(3-4/Tsim*t)*cos(w*t)+w^2*12/Tsim*sin(w*t)];
        Uddot(:,:,3) = [-w^3*(3-4/Tsim*t)*cos(w*t-2*pi/3)+w^2*12/Tsim*sin(w*t-2*pi/3); 0; -w^3*(3-4/Tsim*t)*cos(w*t-2*pi/3)+w^2*12/Tsim*sin(w*t-2*pi/3)];
        Uddot(:,:,4) = [-w^3*(3-4/Tsim*t)*cos(w*t-4*pi/3)+w^2*12/Tsim*sin(w*t-4*pi/3); 0; -w^3*(3-4/Tsim*t)*cos(w*t-4*pi/3)+w^2*12/Tsim*sin(w*t-4*pi/3)];
        
    elseif Q == 2
        % Define desired motion    
        Pd(:,:,1) = [                            0; v*t;                             0];
        Pd(:,:,2) = [       (-1+4/Tsim*t)*sin(w*t); v*t;        (-1+4/Tsim*t)*cos(w*t)];
        Pd(:,:,3) = [(-1+4/Tsim*t)*sin(w*t-2*pi/3); v*t; (-1+4/Tsim*t)*cos(w*t-2*pi/3)];
        Pd(:,:,4) = [(-1+4/Tsim*t)*sin(w*t-4*pi/3); v*t; (-1+4/Tsim*t)*cos(w*t-4*pi/3)];

        Vd(:,:,1) = [                                                    0; v;                                                      0];
        Vd(:,:,2) = [              w*(-1+4/Tsim*t)*cos(w*t)+4/Tsim*sin(w*t); v;               -w*(-1+4/Tsim*t)*sin(w*t)+4/Tsim*cos(w*t)];
        Vd(:,:,3) = [w*(-1+4/Tsim*t)*cos(w*t-2*pi/3)+4/Tsim*sin(w*t-2*pi/3); v; -w*(-1+4/Tsim*t)*sin(w*t-2*pi/3)+4/Tsim*cos(w*t-2*pi/3)];
        Vd(:,:,4) = [w*(-1+4/Tsim*t)*cos(w*t-4*pi/3)+4/Tsim*sin(w*t-4*pi/3); v; -w*(-1+4/Tsim*t)*sin(w*t-4*pi/3)+4/Tsim*cos(w*t-4*pi/3)];

        Ud(:,:,1) = [                                                         0; 0;                                                         0];
        Ud(:,:,2) = [              -w^2*(-1+4/Tsim*t)*sin(w*t)+w*8/Tsim*cos(w*t); 0;               -w^2*(-1+4/Tsim*t)*cos(w*t)-w*8/Tsim*sin(w*t)];
        Ud(:,:,3) = [-w^2*(-1+4/Tsim*t)*sin(w*t-2*pi/3)+w*8/Tsim*cos(w*t-2*pi/3); 0; -w^2*(-1+4/Tsim*t)*cos(w*t-2*pi/3)-w*8/Tsim*sin(w*t-2*pi/3)];
        Ud(:,:,4) = [-w^2*(-1+4/Tsim*t)*sin(w*t-4*pi/3)+w*8/Tsim*cos(w*t-4*pi/3); 0; -w^2*(-1+4/Tsim*t)*cos(w*t-4*pi/3)-w*8/Tsim*sin(w*t-4*pi/3)];

        Uddot(:,:,1) = [                                                            0; 0;                                                             0];
        Uddot(:,:,2) = [              -w^3*(-1+4/Tsim*t)*cos(w*t)-w^2*12/Tsim*sin(w*t); 0;               -w^3*(-1+4/Tsim*t)*cos(w*t)-w^2*12/Tsim*sin(w*t)];
        Uddot(:,:,3) = [-w^3*(-1+4/Tsim*t)*cos(w*t-2*pi/3)-w^2*12/Tsim*sin(w*t-2*pi/3); 0; -w^3*(-1+4/Tsim*t)*cos(w*t-2*pi/3)-w^2*12/Tsim*sin(w*t-2*pi/3)];
        Uddot(:,:,4) = [-w^3*(-1+4/Tsim*t)*cos(w*t-4*pi/3)-w^2*12/Tsim*sin(w*t-4*pi/3); 0; -w^3*(-1+4/Tsim*t)*cos(w*t-4*pi/3)-w^2*12/Tsim*sin(w*t-4*pi/3)];
        
    elseif Q == 3
        % Define desired motion    
        Pd(:,:,1) = [                 0; v*t;                   0];
        Pd(:,:,2) = [    2*cos(w*t) - 1; v*t;            sin(w*t)];
        Pd(:,:,3) = [     cos(w*t-pi/2); v*t; 2*sin(w*t-pi/2) + 1];
        Pd(:,:,4) = [ 2*cos(w*t-pi) + 1; v*t;         sin(w*t-pi)];

        Vd(:,:,1) = [               0; v;                 0];
        Vd(:,:,2) = [   -2*sin(w*t)*w; v;        cos(w*t)*w];
        Vd(:,:,3) = [-sin(w*t-pi/2)*w; v; 2*cos(w*t-pi/2)*w];
        Vd(:,:,4) = [-2*sin(w*t-pi)*w; v;     cos(w*t-pi)*w];

        Ud(:,:,1) = [                 0; 0;                    0];
        Ud(:,:,2) = [   -2*cos(w*t)*w^2; 0;        -sin(w*t)*w^2];
        Ud(:,:,3) = [-cos(w*t-pi/2)*w^2; 0; -2*sin(w*t-pi/2)*w^2];
        Ud(:,:,4) = [-2*cos(w*t-pi)*w^2; 0;     -sin(w*t-pi)*w^2];

        Uddot(:,:,1) = [                 0; 0;                    0];
        Uddot(:,:,2) = [    2*sin(w*t)*w^3; 0;        -cos(w*t)*w^3];
        Uddot(:,:,3) = [ sin(w*t-pi/2)*w^3; 0; -2*cos(w*t-pi/2)*w^3];
        Uddot(:,:,4) = [ 2*sin(w*t-pi)*w^3; 0;     -cos(w*t-pi)*w^3];
        
    elseif Q == 4
        % Define desired motion    
        Pd(:,:,1) = [                 0; v*t;                   0];
        Pd(:,:,2) = [    2*cos(w*t) - 1; v*t;            sin(w*t)];
        Pd(:,:,3) = [     cos(w*t-pi/2); v*t; 2*sin(w*t-pi/2) + 1];
        Pd(:,:,4) = [ 2*cos(w*t-pi) + 1; v*t;         sin(w*t-pi)];

        Vd(:,:,1) = [               0; v;                 0];
        Vd(:,:,2) = [   -2*sin(w*t)*w; v;        cos(w*t)*w];
        Vd(:,:,3) = [-sin(w*t-pi/2)*w; v; 2*cos(w*t-pi/2)*w];
        Vd(:,:,4) = [-2*sin(w*t-pi)*w; v;     cos(w*t-pi)*w];

        Ud(:,:,1) = [                 0; 0;                    0];
        Ud(:,:,2) = [   -2*cos(w*t)*w^2; 0;        -sin(w*t)*w^2];
        Ud(:,:,3) = [-cos(w*t-pi/2)*w^2; 0; -2*sin(w*t-pi/2)*w^2];
        Ud(:,:,4) = [-2*cos(w*t-pi)*w^2; 0;     -sin(w*t-pi)*w^2];

        Uddot(:,:,1) = [                 0; 0;                    0];
        Uddot(:,:,2) = [    2*sin(w*t)*w^3; 0;        -cos(w*t)*w^3];
        Uddot(:,:,3) = [ sin(w*t-pi/2)*w^3; 0; -2*cos(w*t-pi/2)*w^3];
        Uddot(:,:,4) = [ 2*sin(w*t-pi)*w^3; 0;     -cos(w*t-pi)*w^3];
    end
   
    U    = Ud;
    
    % Control system for each agent
    for i = 1:na

        % Position control
        for j = N{i}
            pij = P(1,:,:,j)' - P(1,:,:,i)';
            gij = pij/norm(pij);
            pi_gij = eye(3) - gij*gij.';
            pij_d = Pd(:,:,j) - Pd(:,:,i);
            vti = V(1,:,:,i)' - Vd(:,:,i);
            usum = (-kp(i)*pi_gij*pij_d - kd(i)*vti);
            U(:,:,i) = U(:,:,i) + usum;
        end
        
        TRd = -(m(i)*U(:,:,i) - m(i)*g*e3);
        T   = norm(TRd);

        Rde3(:,i)  = TRd/T;
        
    end
end