function reduce_grid(divisions)
ax = gca;

xyz = 'XYZ';

for i = 1:length(divisions)
    lim = ax.([xyz(i),'Lim']);
    tick = ax.([xyz(i),'Tick']);
    dtick = tick(2) - tick(1);
    rot = ax.([xyz(i),'TickLabelRotation']);
    ax.([xyz(i),'Tick']) = (tick(1)-dtick):dtick/divisions(i):(tick(end)+dtick);
    labels = cell(length(ax.([xyz(i),'Tick'])),1);
    for j=2:(length(labels)-1)
        if mod(j-1,divisions(i)) == 0
            labels{j} = tick(round((j-1)/divisions(i)));
        else
            labels{j} = '';
        end
    end
    ax.([xyz(i),'TickLabel']) = labels;
    ax.([xyz(i),'TickLabelRotation']) = rot;
    ax.([xyz(i),'Lim']) = lim;
end

% ylim = ax.YLim;
% zlim = ax.ZLim;
% ytick = ax.YTick;
% ztick = ax.ZTick;
% ax.YTickLabel
% ax.ZTickLabel
end