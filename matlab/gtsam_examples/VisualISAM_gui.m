function varargout = VisualISAM_gui(varargin)
% VisualISAM_gui: runs VisualSLAM iSAM demo in GUI
%   Interface is defined by VisualISAM_gui.fig
%   You can run this file directly, but won't have access to globals
%   By running ViusalISAMDemo, you see all variables in command prompt
% Authors: Duy Nguyen Ta

% Last Modified by GUIDE v2.5 13-Jun-2012 23:15:43

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
function VisualISAM_gui_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% varargin   command line arguments to VisualISAM_gui (see VARARGIN)

% Choose default command line output for VisualISAM_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = VisualISAM_gui_OutputFcn(hObject, ~, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% Get default command line output from handles structure
varargout{1} = handles.output;


%----------------------------------------------------------
% Convenient functions
%----------------------------------------------------------
function showFramei(hObject, handles)
global frame_i
set(handles.frameStatus, 'String', sprintf('Frame: %d',frame_i));
drawnow
guidata(hObject, handles);

function showWaiting(handles, status)
set(handles.waitingStatus,'String', status);
drawnow
guidata(handles.waitingStatus, handles);

function triangle = chooseDataset(handles)
str = cellstr(get(handles.dataset,'String'));
sel = get(handles.dataset,'Value');
switch str{sel}
    case 'triangle'
        triangle = true;
    case 'cube'
        triangle = false;
end

function initOptions(handles)

global options

% Data options
options.triangle = chooseDataset(handles);
options.nrCameras = str2num(get(handles.numCamEdit,'String'));
options.showImages = get(handles.showImagesCB,'Value');

% iSAM Options
options.hardConstraint = get(handles.hardConstraintCB,'Value');
options.pointPriors = get(handles.pointPriorsCB,'Value');
options.batchInitialization = get(handles.batchInitCB,'Value');
%options.reorderInterval = str2num(get(handles.reorderIntervalEdit,'String'));
options.alwaysRelinearize = get(handles.alwaysRelinearizeCB,'Value');

% Display Options
options.saveDotFile = get(handles.saveGraphCB,'Value');
options.printStats = get(handles.printStatsCB,'Value');
options.drawInterval = str2num(get(handles.drawInterval,'String'));
options.cameraInterval = str2num(get(handles.cameraIntervalEdit,'String'));
options.drawTruePoses = get(handles.drawTruePosesCB,'Value');
options.saveFigures = get(handles.saveFiguresCB,'Value');
options.saveDotFiles = get(handles.saveGraphsCB,'Value');

%----------------------------------------------------------
% Callback functions for GUI elements
%----------------------------------------------------------

% --- Executes during object creation, after setting all properties.
function dataset_CreateFcn(hObject, ~, handles)
% handles    empty - handles not created until after all CreateFcns called
% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in dataset.
function dataset_Callback(hObject, ~, handles)
% Hints: contents = cellstr(get(hObject,'String')) returns dataset contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dataset


% --- Executes during object creation, after setting all properties.
function numCamEdit_CreateFcn(hObject, ~, handles)
% Hint: edit controls usually have a white background on Windows.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function numCamEdit_Callback(hObject, ~, handles)
% Hints: get(hObject,'String') returns contents of numCamEdit as text
%        str2double(get(hObject,'String')) returns contents of numCamEdit as a double


% --- Executes on button press in showImagesCB.
function showImagesCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of showImagesCB


% --- Executes on button press in hardConstraintCB.
function hardConstraintCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of hardConstraintCB


% --- Executes on button press in pointPriorsCB.
function pointPriorsCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of pointPriorsCB


% --- Executes during object creation, after setting all properties.
function batchInitCB_CreateFcn(hObject, eventdata, handles)
set(hObject,'Value',1);

% --- Executes on button press in batchInitCB.
function batchInitCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of batchInitCB


% --- Executes on button press in alwaysRelinearizeCB.
function alwaysRelinearizeCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of alwaysRelinearizeCB


% --- Executes during object creation, after setting all properties.
function reorderIntervalText_CreateFcn(hObject, ~, handles)
% Hint: edit controls usually have a white background on Windows.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function reorderIntervalEdit_CreateFcn(hObject, ~, handles)
% Hint: edit controls usually have a white background on Windows.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function drawInterval_CreateFcn(hObject, ~, handles)
% Hint: edit controls usually have a white background on Windows.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function drawInterval_Callback(hObject, ~, handles)
% Hints: get(hObject,'String') returns contents of drawInterval as text
%        str2double(get(hObject,'String')) returns contents of drawInterval as a double


function cameraIntervalEdit_Callback(hObject, ~, handles)
% Hints: get(hObject,'String') returns contents of cameraIntervalEdit as text
%        str2double(get(hObject,'String')) returns contents of cameraIntervalEdit as a double


% --- Executes during object creation, after setting all properties.
function cameraIntervalEdit_CreateFcn(hObject, ~, handles)
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in saveGraphCB.
function saveGraphCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of saveGraphCB


% --- Executes on button press in printStatsCB.
function printStatsCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of printStatsCB


% --- Executes on button press in drawTruePosesCB.
function drawTruePosesCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of drawTruePosesCB


% --- Executes on button press in saveFiguresCB.
function saveFiguresCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of saveFiguresCB


% --- Executes on button press in saveGraphsCB.
function saveGraphsCB_Callback(hObject, ~, handles)
% Hint: get(hObject,'Value') returns toggle state of saveGraphsCB

% --- Executes on button press in intializeButton.
function intializeButton_Callback(hObject, ~, handles)

global frame_i truth data noiseModels isam result nextPoseIndex options

% initialize global options
initOptions(handles)

% Generate Data
[data,truth] = gtsam.VisualISAMGenerateData(options);

% Initialize and plot
[noiseModels,isam,result,nextPoseIndex] = gtsam.VisualISAMInitialize(data,truth,options);
cla
gtsam.VisualISAMPlot(truth, data, isam, result, options)
frame_i = 2;
showFramei(hObject, handles)


% --- Executes on button press in runButton.
function runButton_Callback(hObject, ~, handles)
global frame_i truth data noiseModels isam result nextPoseIndex options
while (frame_i<size(truth.cameras,2))
    frame_i = frame_i+1;
    showFramei(hObject, handles)
    [isam,result,nextPoseIndex] = gtsam.VisualISAMStep(data,noiseModels,isam,result,truth,nextPoseIndex);
    if mod(frame_i,options.drawInterval)==0
        showWaiting(handles, 'Computing marginals...');
        gtsam.VisualISAMPlot(truth, data, isam, result, options)
        showWaiting(handles, '');
    end
end


% --- Executes on button press in stepButton.
function stepButton_Callback(hObject, ~, handles)
global frame_i truth data noiseModels isam result nextPoseIndex options
if (frame_i<size(truth.cameras,2))
    frame_i = frame_i+1;
    showFramei(hObject, handles)
    [isam,result,nextPoseIndex] = gtsam.VisualISAMStep(data,noiseModels,isam,result,truth,nextPoseIndex);
    showWaiting(handles, 'Computing marginals...');
    gtsam.VisualISAMPlot(truth, data, isam, result, options)
    showWaiting(handles, '');
end
