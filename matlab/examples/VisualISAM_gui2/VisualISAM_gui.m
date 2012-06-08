function varargout = VisualISAM_gui(varargin)
% VISUALISAM_GUI MATLAB code for VisualISAM_gui.fig
%      VISUALISAM_GUI, by itself, creates a new VISUALISAM_GUI or raises the existing
%      singleton*.
%
%      H = VISUALISAM_GUI returns the handle to a new VISUALISAM_GUI or the handle to
%      the existing singleton*.
%
%      VISUALISAM_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VISUALISAM_GUI.M with the given input arguments.
%
%      VISUALISAM_GUI('Property','Value',...) creates a new VISUALISAM_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before VisualISAM_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to VisualISAM_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help VisualISAM_gui

% Last Modified by GUIDE v2.5 08-Jun-2012 14:00:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @VisualISAM_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @VisualISAM_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before VisualISAM_gui is made visible.
function VisualISAM_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to VisualISAM_gui (see VARARGIN)

% Choose default command line output for VisualISAM_gui
%% Data Options
handles.TRIANGLE = true;
handles.NCAMERAS = 20;
handles.SHOW_IMAGES = false;

%% iSAM Options
handles.HARD_CONSTRAINT = false;
handles.POINT_PRIORS = false;
handles.BATCH_INIT = true;
handles.REORDER_INTERVAL=10;
handles.ALWAYS_RELINEARIZE = false;

%% Display Options
handles.SAVE_GRAPH = false;
handles.PRINT_STATS = true;
handles.DRAW_INTERVAL = 4;
handles.CAMERA_INTERVAL = 1;
handles.DRAW_TRUE_POSES = false;
handles.SAVE_FIGURES = false;
handles.SAVE_GRAPHS = false;

handles = vData(handles);
handles = vInit(handles);
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes VisualISAM_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = VisualISAM_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function handles = initialize(handles)
    handles=vData(handles);
    handles=vInit(handles);
    handles.result = {};
    cla;
    handles
    
% --- Executes on button press in intializeButton.
function intializeButton_Callback(hObject, eventdata, handles)
% hObject    handle to intializeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.DRAW_INTERVAL = str2num(get(handles.drawInterval,'String')) ;
    handles.NCAMERAS = str2num(get(handles.numCamEdit,'String')) ;
    handles = initialize(handles)
    guidata(hObject,handles)

% --- Executes on button press in stepButton.
function stepButton_Callback(hObject, eventdata, handles)
% hObject    handle to stepButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    if (handles.frame_i<handles.NCAMERAS)
        handles.frame_i = handles.frame_i+1;
        sprintf('Frame %d:', handles.frame_i)
        handles = vStep(handles);
        guidata(hObject,handles)
    end

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
    str = cellstr(get(hObject,'String'));
    sel = get(hObject,'Value');
    switch str{sel}
        case 'triangle'
            handles.TRIANGLE = true
        case 'cube'
            handles.TRIANGLE = false
    end
    handles = initialize(handles);
    guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in backButton.
function backButton_Callback(hObject, eventdata, handles)
% hObject    handle to backButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    sprintf('Not yet implemented')

% --- Executes on button press in runButton.
function runButton_Callback(hObject, eventdata, handles)
% hObject    handle to runButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    sprintf('Not yet implemented')

% --- Executes on button press in stopButton.
function stopButton_Callback(hObject, eventdata, handles)
% hObject    handle to stopButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    sprintf('Not yet implemented')


% --- Executes on button press in plotButton.
function plotButton_Callback(hObject, eventdata, handles)
% hObject    handle to plotButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    vPlot(handles);


function drawInterval_Callback(hObject, eventdata, handles)
% hObject    handle to drawInterval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of drawInterval as text
%        str2double(get(hObject,'String')) returns contents of drawInterval as a double
    handles.DRAW_INTERVAL = str2num(get(hObject,'String')) ;
    handles
    guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function drawInterval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawInterval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function numCamEdit_Callback(hObject, eventdata, handles)
% hObject    handle to numCamEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of numCamEdit as text
%        str2double(get(hObject,'String')) returns contents of numCamEdit as a double
    handles.NCAMERAS = str2num(get(hObject,'String')) ;
    handles
    guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function numCamEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to numCamEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
