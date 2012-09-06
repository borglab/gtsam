function varargout = gtsamExamples(varargin)
% GTSAMEXAMPLES MATLAB code for gtsamExamples.fig
%      GTSAMEXAMPLES, by itself, creates a new GTSAMEXAMPLES or raises the existing
%      singleton*.
%
%      H = GTSAMEXAMPLES returns the handle to a new GTSAMEXAMPLES or the handle to
%      the existing singleton*.
%
%      GTSAMEXAMPLES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GTSAMEXAMPLES.M with the given input arguments.
%
%      GTSAMEXAMPLES('Property','Value',...) creates a new GTSAMEXAMPLES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gtsamExamples_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gtsamExamples_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gtsamExamples

% Last Modified by GUIDE v2.5 03-Sep-2012 13:34:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gtsamExamples_OpeningFcn, ...
                   'gui_OutputFcn',  @gtsamExamples_OutputFcn, ...
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

% --- Executes just before gtsamExamples is made visible.
function gtsamExamples_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gtsamExamples (see VARARGIN)

% Choose default command line output for gtsamExamples
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

OdometryExample;

% --- Outputs from this function are returned to the command line.
function varargout = gtsamExamples_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)

% --- Executes on button press in Odometry.
function Odometry_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
OdometryExample;
echo off

% --- Executes on button press in Localization.
function Localization_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
LocalizationExample;
echo off

% --- Executes on button press in Pose2SLAM.
function Pose2SLAM_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
Pose2SLAMExample
echo off

% --- Executes on button press in Pose2SLAMCircle.
function Pose2SLAMCircle_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
Pose2SLAMExample_circle
echo off

% --- Executes on button press in Pose2SLAMManhattan.
function Pose2SLAMManhattan_Callback(hObject, eventdata, handles)
axes(handles.axes3);
Pose2SLAMExample_graph

% --- Executes on button press in Pose3SLAM.
function Pose3SLAM_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
Pose3SLAMExample
echo off

% --- Executes on button press in Pose3SLAMSphere.
function Pose3SLAMSphere_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
Pose3SLAMExample_graph
echo off

% --- Executes on button press in PlanarSLAM.
function PlanarSLAM_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
PlanarSLAMExample
echo off

% --- Executes on button press in PlanarSLAMSampling.
function PlanarSLAMSampling_Callback(hObject, eventdata, handles)
axes(handles.axes3);
PlanarSLAMExample_sampling

% --- Executes on button press in PlanarSLAMGraph.
function PlanarSLAMGraph_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
PlanarSLAMExample_graph
echo off

% --- Executes on button press in SFM.
function SFM_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
SFMExample
echo off

% --- Executes on button press in VisualISAM.
function VisualISAM_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
VisualISAMExample
echo off

% --- Executes on button press in StereoVO.
function StereoVO_Callback(hObject, eventdata, handles)
axes(handles.axes3);
echo on
StereoVOExample
echo off

% --- Executes on button press in StereoVOLarge.
function StereoVOLarge_Callback(hObject, eventdata, handles)
axes(handles.axes3);
StereoVOExample_large
