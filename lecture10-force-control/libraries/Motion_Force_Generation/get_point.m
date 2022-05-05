function data = get_point(fig,hObject,handles)
% to store the data
X = [];
% flag for signaling that the demonstration has ended
finished = 0;

% select our figure as gcf
figure(fig);
hold on;
% disable any figure modes
zoom off 
rotate3d off
pan off
brush off  
datacursormode off 

set(fig,'WindowButtonDownFcn',@(h,e)button_clicked(h,e));
set(fig,'WindowButtonUpFcn',[]);
set(fig,'WindowButtonMotionFcn',[]);
% set(fig,'Pointer','crosshair');


% wait until demonstration is finished
while(~finished && ~handles.stop && ~handles.updateSurface)
    handles = guidata(hObject);
    pause(0.1);
end
% set the return value
if(~handles.stop && ~handles.updateSurface)
    data = X(1,1:2)';
else
    data = [];
end
% set(fig,'Pointer','arrow');
return

    function ret = button_clicked(h,e)
        if(strcmp(get(gcf,'SelectionType'),'normal'))
            X = get(gca,'Currentpoint');
            set(gcf,'WindowButtonUpFcn', @stop_demonstration);
        end
    end

    function ret = stop_demonstration(h,e)
        set(gcf,'WindowButtonMotionFcn',[]);
        set(gcf,'WindowButtonUpFcn',[]);
        set(gcf,'WindowButtonDownFcn',[]);
        finished = 1;
    end

    
end
