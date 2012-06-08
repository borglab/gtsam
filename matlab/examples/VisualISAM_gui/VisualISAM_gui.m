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

% Last Modified by GUIDE v2.5 08-Jun-2012 03:28:25

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

% handles.data = VisualISAMData_triangle();
% handles.frame_i = 3;
% handles.isam = visualSLAMISAM;
% handles.results = visualSLAMValues;

% Choose default command line output for VisualISAM_gui
handles.selectedDataset = 'triangle';
handles = initialize(handles);
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes VisualISAM_gui wait for user response (see UIRESUME)
% uiwait(handles.Dataset);


% --- Outputs from this function are returned to the command line.
function varargout = VisualISAM_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in generateButton.
function generateButton_Callback(hObject, eventdata, handles)
% hObject    handle to generateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.data = VisualISAMData_triangle();
    guidata(hObject,handles)

% --- Initialize a new dataset
function handles=initialize(handles)
    handles.selectedDataset
    switch handles.selectedDataset
        case 'cube' 
            handles.data = VisualISAMData_cube();
        case 'triangle'
            handles.data = VisualISAMData_triangle();
    end
    handles.data
    handles.results = {}
    [handles.isam handles.results{2}] = VisualISAMInitialize(handles.data);
    handles.frame_i=2;
    sprintf('Frame 1,2:')
    handles.results{2}.estimates
    cla(handles.resultAxes);
	VisualISAMPlot(handles.results{handles.frame_i}, handles.data)
    view([36 34])
    colormap('hot')


% --- Executes on button press in intializeButton.
function intializeButton_Callback(hObject, eventdata, handles)
% hObject    handle to intializeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles=initialize(handles)
    guidata(hObject,handles)
    

% --- Executes on button press in stepButton.
function stepButton_Callback(hObject, eventdata, handles)
% hObject    handle to stepButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    if (handles.frame_i<size(handles.data.cameras,2))
        handles.frame_i = handles.frame_i+1;
        sprintf('Frame %d:', handles.frame_i)
        if (handles.frame_i > size(handles.results,2))
            [handles.isam handles.results{handles.frame_i}] = ...
                VisualISAMStep(handles.frame_i, handles.isam, ...
                    handles.data, handles.results{handles.frame_i-1}); 
        end
        handles.results{handles.frame_i}.estimates
        cla(handles.resultAxes);
        VisualISAMPlot(handles.results{handles.frame_i}, handles.data)
        guidata(hObject,handles)
    else
        sprintf('Frame %d:', handles.frame_i) 
        sprintf('No more frame!')
    end
    

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
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
    handles.selectedDataset = str{sel}
    handles=initialize(handles)
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


% --- Executes during object creation, after setting all properties.
function Dataset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Dataset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in backButton.
function backButton_Callback(hObject, eventdata, handles)
% hObject    handle to backButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    if (handles.frame_i>2)
        handles.frame_i = handles.frame_i-1;
        sprintf('Frame %d:', handles.frame_i)
        handles.results{handles.frame_i}.estimates
        cla(handles.resultAxes);
        VisualISAMPlot(handles.results{handles.frame_i}, handles.data)
        guidata(hObject,handles)
    else
        sprintf('No more frame!');
    end

% --- Executes on button press in runButton.
function runButton_Callback(hObject, eventdata, handles)
% hObject    handle to runButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    for i=handles.frame_i+1:size(handles.data.cameras,2)
        if (i > size(handles.results,2))
            [handles.isam handles.results{i}] = ...
                VisualISAMStep(i, handles.isam, ...
                    handles.data, handles.results{i-1}); 
        end
        handles.results{i}.estimates
        cla(handles.resultAxes);
        VisualISAMPlot(handles.results{i}, handles.data)
    end
    handles.frame_i = size(handles.data.cameras,2);
	sprintf('Frame %d:', handles.frame_i)
    guidata(hObject,handles)
    

% --- Executes on button press in stopButton.
function stopButton_Callback(hObject, eventdata, handles)
% hObject    handle to stopButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    sprintf('Not yet implemented')


function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function odoTrans_Callback(hObject, eventdata, handles)
% hObject    handle to odoTrans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of odoTrans as text
%        str2double(get(hObject,'String')) returns contents of odoTrans as a double


% --- Executes during object creation, after setting all properties.
function odoTrans_CreateFcn(hObject, eventdata, handles)
% hObject    handle to odoTrans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function measNoise_Callback(hObject, eventdata, handles)
% hObject    handle to measNoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of measNoise as text
%        str2double(get(hObject,'String')) returns contents of measNoise as a double


% --- Executes during object creation, after setting all properties.
function measNoise_CreateFcn(hObject, eventdata, handles)
% hObject    handle to measNoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posePriorRot_Callback(hObject, eventdata, handles)
% hObject    handle to posePriorRot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posePriorRot as text
%        str2double(get(hObject,'String')) returns contents of posePriorRot as a double


% --- Executes during object creation, after setting all properties.
function posePriorRot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posePriorRot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posePriorTrans_Callback(hObject, eventdata, handles)
% hObject    handle to posePriorTrans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posePriorTrans as text
%        str2double(get(hObject,'String')) returns contents of posePriorTrans as a double


% --- Executes during object creation, after setting all properties.
function posePriorTrans_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posePriorTrans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pointPrior_Callback(hObject, eventdata, handles)
% hObject    handle to pointPrior (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pointPrior as text
%        str2double(get(hObject,'String')) returns contents of pointPrior as a double


% --- Executes during object creation, after setting all properties.
function pointPrior_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pointPrior (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function odoRot_Callback(hObject, eventdata, handles)
% hObject    handle to odoRot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of odoRot as text
%        str2double(get(hObject,'String')) returns contents of odoRot as a double


% --- Executes during object creation, after setting all properties.
function odoRot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to odoRot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
