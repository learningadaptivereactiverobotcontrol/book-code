function P = get_mouse_curve(fig,hObject,handles)
  % GET_PENCIL_CURVE Get a curve (sequence of points) from the user by dragging
  % on the current plot window
  %
  % P = get_pencil_curve()
  % P = get_pencil_curve(f)
  % 
  % Inputs:
  %   f  figure id
  % Outputs:
  %   P  #P by 2 list of point positions
  %
  %

  % Get the input figure or get current one (creates new one if none exist)
  figure(fig);

  set(fig,'windowbuttondownfcn',@ondown);
  set(fig,'keypressfcn',        @onkeypress);
  % living variables
  P = [];
  p = [];

  % loop until mouse up or ESC is pressed
  done = false;
%   set(fig,'Pointer','crosshair');
  while(~done && ~handles.stop)
    handles = guidata(hObject);
    drawnow;
  end
%   set(fig,'Pointer','arrow');
  % We've been also gathering Z coordinate, drop it
  if(~handles.stop)
    P = P(:,1:2);
  else
      P = [];
  end

  % Callback for mouse press
  function ondown(src,ev)
    % Tell window that we'll handle drag and up events
    set(gcf,'windowbuttonmotionfcn', @ondrag);
    set(gcf,'windowbuttonupfcn',     @onup);
    append_current_point();
  end

  % Callback for mouse drag
  function ondrag(src,ev)
    append_current_point();
  end

  % Callback for mouse release
  function onup(src,ev)
    % Tell window to handle down, drag and up events itself
    finish();
  end

  function onkeypress(src,ev)
    % escape character id
    ESC = char(27);
    switch ev.Character
    case ESC
      finish();
    otherwise
      error(['Unknown key: ' ev.Character]);
    end
  end

  function append_current_point()
    % get current mouse position
    cp = get(gca,'currentpoint');
    % append to running list
    P = [P;cp(1,:)];
    if isempty(p)
      % init plot
      hold on;
      p = plot(P(:,1),P(:,2));
      hold off;
    else
      % update plot
      set(p,'Xdata',P(:,1),'Ydata',P(:,2));
    end
  end

  function finish()
    done = true;
    set(gcf,'windowbuttonmotionfcn','');
    set(gcf,'windowbuttonupfcn','');
    set(gcf,'windowbuttondownfcn','');
    set(gcf,'keypressfcn','');
  end

end