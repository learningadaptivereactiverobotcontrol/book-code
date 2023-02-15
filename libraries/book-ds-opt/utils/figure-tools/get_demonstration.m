function [data, hp] = get_demonstration(fig,varargin)

% option to delete/not delete data aftere finshed demonstration
delete_trace = 1;
if (nargin>1)
    delete_trace = varargin{1};
end
    

% to store the data
X = [];
% flag for signaling that the demonstration has ended
finished = 0;

% select our figure as gcf
figure(fig);
hold on
% disable any figure modes
zoom off
rotate3d off
pan off
brush off
datacursormode off

set(fig,'WindowButtonDownFcn',@(h,e)button_clicked(h,e));
set(fig,'WindowButtonUpFcn',[]);
set(fig,'WindowButtonMotionFcn');
set(fig,'Pointer','circle');

hp = gobjects(0);

% wait until demonstration is finished
while(~finished)
    pause(0.1);
end
% set the return value
data = X;
set(fig,'Pointer','arrow');
return

   

    function ret = button_clicked(h,e)
        if(strcmp(get(gcf,'SelectionType'),'normal'))
            start_demonstration();
        end
    end

    function ret = start_demonstration()
        disp('started demonstration')
        set(gcf,'WindowButtonUpFcn',@stop_demonstration);
        set(gcf,'WindowButtonMotionFcn',@record_current_point);
        ret = 1;
        tic;
    end

    function ret = stop_demonstration(h,e)
        disp('stopped demonstration')
        set(gcf,'WindowButtonMotionFcn',[]);
        set(gcf,'WindowButtonUpFcn',[]);
        set(gcf,'WindowButtonDownFcn',[]);
        if(delete_trace)
            delete(hp);
        end
        finished = 1;
    end

    function ret = record_current_point(h,e)
        x = get(gca,'Currentpoint');
        x = x(1,1:2)';
        x = [x;toc];
        X = [X, x];
        hp = [hp, plot(x(1),x(2),'r.','markersize',20)];
    end
end
