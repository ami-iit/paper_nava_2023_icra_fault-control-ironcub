function varargout = ironcubControlGui(varargin)
% IRONCUBCONTROLGUI MATLAB code for ironcubControlGui.fig
%      IRONCUBCONTROLGUI, by itself, creates a new IRONCUBCONTROLGUI or raises the existing
%      singleton*.
%
%      H = IRONCUBCONTROLGUI returns the handle to a new IRONCUBCONTROLGUI or the handle to
%      the existing singleton*.
%
%      IRONCUBCONTROLGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IRONCUBCONTROLGUI.M with the given input arguments.
%
%      IRONCUBCONTROLGUI('Property','Value',...) creates a new IRONCUBCONTROLGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ironcubControlGui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ironcubControlGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ironcubControlGui

% Last Modified by GUIDE v2.5 11-Mar-2020 00:34:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ironcubControlGui_OpeningFcn, ...
                   'gui_OutputFcn',  @ironcubControlGui_OutputFcn, ...
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


% --- Executes just before ironcubControlGui is made visible.
function ironcubControlGui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ironcubControlGui (see VARARGIN)

% Choose default command line output for ironcubControlGui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ironcubControlGui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ironcubControlGui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_up.
function pushbutton_up_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','1')

% --- Executes on button press in pushbutton_down.
function pushbutton_down_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','2')

% --- Executes on button press in pushbutton_right.
function pushbutton_right_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','3')

% --- Executes on button press in pushbutton_left.
function pushbutton_left_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','4')

% --- Executes on button press in pushbutton_pitch_plus.
function pushbutton_pitch_plus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_pitch_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','5')

% --- Executes on button press in pushbutton_pitch_minus.
function pushbutton_pitch_minus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_pitch_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','6')

% --- Executes on button press in pushbutton_yaw_plus.
function pushbutton_yaw_plus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_yaw_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','7')

% --- Executes on button press in pushbutton_yaw_minus.
function pushbutton_yaw_minus_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_yaw_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','8')

% --- Executes on button press in pushbutton_takeoff.
function pushbutton_takeoff_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_takeoff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','9')

% --- Executes on button press in pushbutton_land.
function pushbutton_land_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_land (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','10')

% --- Executes on button press in pushbutton_front.
function pushbutton_front_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_front (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','11')

% --- Executes on button press in pushbutton_back.
function pushbutton_back_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','12')

% --- Executes on button press in pushbutton_turbo_on.
function pushbutton_turbo_on_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_turbo_on (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','13')

% --- Executes on button press in pushbutton_turbo_off.
function pushbutton_turbo_off_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_turbo_off (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.93,0.69,0.13]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.9,0.9,0.9]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','14')

% --- Executes on button press in pushbutton_reset.
function pushbutton_reset_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_up,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_down,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_right,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_left,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_pitch_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_plus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_yaw_minus,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_takeoff,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_land,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_front,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_back,'Backgroundcolor',[0.65,0.65,0.65]);
set(handles.pushbutton_turbo_on,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_turbo_off,'Backgroundcolor',[0.9,0.9,0.9]);
set(handles.pushbutton_reset,'Backgroundcolor',[0.93,0.69,0.13]);
set_param('momentumBasedFlight/FLIGHT CONTROLLER (CORE) V3.0/Input_GUI','Value','0')
