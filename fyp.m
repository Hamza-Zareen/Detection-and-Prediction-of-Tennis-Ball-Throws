function varargout = fyp(varargin)
% FYP MATLAB code for fyp.fig
%      FYP, by itself, creates a new FYP or raises the existing
%      singleton*.
%
%      H = FYP returns the handle to a new FYP or the handle to
%      the existing singleton*.
%
%      FYP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FYP.M with the given input arguments.
%
%      FYP('Property','Value',...) creates a new FYP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before fyp_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to fyp_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help fyp

% Last Modified by GUIDE v2.5 16-Aug-2018 11:18:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @fyp_OpeningFcn, ...
    'gui_OutputFcn',  @fyp_OutputFcn, ...
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

function setvidbrowse(val)
global x
x=val;
function r=getvidbrowse
global x
r=x;
function setFrame(val)
global a
a=val;
function r=getFrame
global a
r=a;

% --- Executes just before fyp is made visible.
function fyp_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to fyp (see VARARGIN)

% Choose default command line output for fyp
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes fyp wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = fyp_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in BrowseVideo.
function BrowseVideo_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseVideo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[a,b]=uigetfile({'*.*','All Files'});
filepath=strcat(b,a);
setvidbrowse(filepath);
% --- Executes on button press in Detection.
function Detection_Callback(hObject, eventdata, handles)
% hObject    handle to Detection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes1);
promptmsg1=strcat('Enter value of start frame it should be < ',getFrame);
promptmsg2=strcat('Enter value of ending frame it should be < ',getFrame);
prompt={promptmsg1,promptmsg2};
dlg_title='input';
num_line=1;
answer=inputdlg(prompt,dlg_title,num_line);
minval=str2double(answer(1));
maxval=str2double(answer(2));
vidObj = VideoReader(getvidbrowse);
figure;
currAxes = axes;
xAxis=0;
yAxis=0;
count=0;
offset=1;
bl=true;
combineimgfalg=true;
f=zeros(1,2);
arr=['A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M' 'N' 'O' 'P' 'Q' 'R' 'S' 'T' 'U' 'V' 'W' 'X' 'Y' 'Z'];
rect=[1.705100000000000e+02,1.505100000000000e+02,8.489800000000000e+02,4.749800000000000e+02];
while hasFrame(vidObj)
    vidFrame = readFrame(vidObj);
    image(vidFrame, 'Parent', currAxes);
    currAxes.Visible = 'off';
    pause(1/vidObj.FrameRate);
    count=count+1;
    if count>minval && count<maxval
        vidFrame=imcrop(vidFrame,rect);
        temp2=vidFrame;
        bw=temp-vidFrame;
        bw=rgb2gray(bw);
        bw=imbinarize(bw);
        structelmnt=strel('disk',3);
        bw=imdilate(bw,structelmnt);
        se = strel('disk',50);
        bw = imclose(bw,se);
        bw = bwareaopen(bw,30);
        bw = imfill(bw,'holes');
        se = strel('disk',1);
        bw = imopen(bw,se);
        bw= bwpropfilt(bw,'Area',[50 1000]);
        %  figure, imshow(bw);
        [B,L] = bwboundaries(bw,'noholes');
        stats = regionprops(L,'Area','Centroid');
        threshold = 0.75;
        
        % loop over the boundaries
        for k = 1:length(B)
            
            % obtain (X,Y) boundary coordinates corresponding to label 'k'
            boundary = B{k};
            
            % compute a simple estimate of the object's perimeter
            delta_sq = diff(boundary).^2;
            perimeter = sum(sqrt(sum(delta_sq,2)));
            
            % obtain the area calculation corresponding to label 'k'
            area = stats(k).Area;
            if area<=180
                % compute the roundness metric
                metric = 4*pi*area/perimeter^2;
                
                % display the results
                metric_string = sprintf('%2.2f',metric);
                
                % mark objects above the threshold with a black circle
                if metric > threshold
                    %                                           figure;
                    axes(handles.axes1);
                    imshow(temp);
                    hold on
                    centroid = stats(k).Centroid;
                    plot(centroid(1),centroid(2),'ko');
                    f=[f;centroid];
                    xAxis=centroid(1);
                    yAxis=centroid(2);
                end
            end
        end
        title(['Metrics closer to 1 indicate that ',...
            'the object is approximately round']);
        bl=false;
        %                     rang=strcat(arr(offset),num2str(36));
        %                     xlswrite('C:\Users\Hamza Zareen\Desktop\federerx.xlsx',xAxis,1,rang);
        %                     xlswrite('C:\Users\Hamza Zareen\Desktop\federery.xlsx',yAxis,1,rang);
        a=xAxis;
        xAxis=0;
        %                     y=0;
        %                     offset = offset +1;
    end
    
    temp=vidFrame;
    
    if bl==true
        temp=imcrop(temp,rect);
        
    end
   
end
figure;
imshow(temp2);
hold on
for idx = 2: length(f)
    plot(f(idx,1),f(idx,2),'ko');
end
title('Combined Results', 'FontSize', 14);

function PredictionFor2DPrediction_Callback(val)
% hObject    handle to StartPrediction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

for i=1:val
    
    rang=strcat('A',num2str(i),':','Z',num2str(i));
    subsetA = xlsread('C:\Users\Hamza Zareen\Desktop\federerx.xlsx',1,rang);
    subsetB = xlsread('C:\Users\Hamza Zareen\Desktop\federery.xlsx',1,rang);
    s=false;
    val1=zeros(0);
    val2=zeros(0);
    for k=1:length(subsetA)
        if subsetA(k)>0 || s==true
            val1=[val1;subsetA(k)];
            val2=[val2;subsetB(k)];
            s=true;
        end
    end
    
    kalmanFilter = []; isTrackInitialized = false;
    detectedLocationsx=val1;
    detectedLocationsy=val2;
    arr=['A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M' 'N' 'O' 'P' 'Q' 'R' 'S' 'T' 'U' 'V' 'W' 'X' 'Y' 'Z'];
    offset=1;
%     axes(handles.axes2);
    hold on;
    ylabel('Location');
    xlabel('Time');
    for idx = 1: length(detectedLocationsx)
        detectedLocation= [detectedLocationsx(idx),detectedLocationsy(idx)];
        isObjectDetected = size(detectedLocation, 1) > 0;
        if detectedLocation == [0,0]
            isObjectDetected = 0;
        end
        if ~isTrackInitialized
            if isObjectDetected
                kalmanFilter = configureKalmanFilter('ConstantAcceleration',detectedLocation(1,:), [1 1 1]*1e5, [25, 10, 10], 25);
                isTrackInitialized = true;
            end
            label = ''; circle = zeros(0,3);
        else
            if isObjectDetected
                predict(kalmanFilter);
                
                trackedLocation = correct(kalmanFilter, detectedLocation(1,:));
                label = 'Corrected';
                
            else
                trackedLocation = predict(kalmanFilter);
                label = 'Predicted';
            end
            d = distance(trackedLocation,detectedLocation);
            title(sprintf('Distance:%f', d));
            plot(idx, detectedLocation,'k+');
            pause(0.2);
            plot(idx, trackedLocation,'ro');
            %          plot(detectedLocation(1), detectedLocation(2),'k+');
            %         pause(0.2);
            %         plot(trackedLocation(1), trackedLocation(2),'ro');
            %         rang=strcat(arr(offset),num2str(i));
            %         xlswrite('C:\Users\Hamza Zareen\Desktop\federerconfigklmx.xlsx',trackedLocation(1),1,rang);
            %          xlswrite('C:\Users\Hamza Zareen\Desktop\federerconfigklmy.xlsx',trackedLocation(2),1,rang);
            %          offset=offset+1;
            %        circle = [trackedLocation, 5];
        end
      
    end
end

% --- Executes on button press in BrowseDatasetX.

% --- Executes on button press in FrameCalculation.
function FrameCalculation_Callback(hObject, eventdata, handles)
% hObject    handle to FrameCalculation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vidObj = VideoReader(getvidbrowse);
Frames=0;
while hasFrame(vidObj)
    vidFrame = readFrame(vidObj);
    Frames=Frames+1;
end
nframe=int2str(Frames);
setFrame(nframe);
numframe=strcat('In this video number of frames are =   ',nframe);
h = msgbox(numframe,'Frames are');


% --- Executes on button press in Predictionx.
function PredictionX_Callback(val)
% hObject    handle to Predictionx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

for i=1:val
    
    rang=strcat('A',num2str(i),':','Z',num2str(i));
    subsetA = xlsread('C:\Users\Hamza Zareen\Desktop\federerx.xlsx',1,rang);
    s=false;
    val1=zeros(0);
    for k=1:length(subsetA)
        if subsetA(k)>0 || s==true
            val1=[val1,subsetA(k)];
            s=true;
        end
    end
    % Generate synthetic data which mimics the 1-D location of a physical object moving at a constant speed.
    arr=['A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M' 'N' 'O' 'P' 'Q' 'R' 'S' 'T' 'U' 'V' 'W' 'X' 'Y' 'Z'];
    offset=1;
    val1=num2cell(val1);
    detectedLocations= val1;
    for idx = 1: length(detectedLocations)
        if detectedLocations{idx} == 0
            detectedLocations{idx} = [];
        end
    end
    %%
%     axes(handles.axes2);
    hold on;
    ylabel('Location');
    xlabel('Time');
    %%
    % Create a 1-D, constant speed Kalman filter when the physical object is first detected. Predict the location of the object based on previous states. If the object is detected at the current time step, use its location to correct the states.
    kalman = [];
    for idx = 1: length(detectedLocations)
        location = detectedLocations{idx};
        if isempty(kalman)
            if ~isempty(location)
                stateModel = [1 1;0 1];
                measurementModel = [1 0];
                kalman = vision.KalmanFilter(stateModel,measurementModel);
                kalman.State = [location, 0];
            end
        else
            trackedLocation = predict(kalman);
            if ~isempty(location)
                plot(idx, location,'k+');
                d = distance(kalman,location);
                title(sprintf('Distance:%f', d));
                trackedLocation = correct(kalman,location);
            else
                %        title('Missing detection');
            end
            %       rang=strcat(arr(offset),num2str(i));
            %                     xlswrite('C:\Users\Hamza Zareen\Desktop\federerklmx.xlsx',trackedLocation,1,rang);
            %
            %                     offset=offset+1;
            pause(0.2);
            plot(idx,trackedLocation,'ro');
        end
    end
end

% --- Executes on button press in Predictiony.
function PredictionY_Callback(val)
% hObject    handle to Predictiony (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

for i=1:val
    rang=strcat('A',num2str(i),':','Z',num2str(i));
    subsetB = xlsread('C:\Users\Hamza Zareen\Desktop\federery.xlsx',1,rang);
    s=false;
    val2=zeros(0);
    for k=1:length(subsetB)
        if subsetB(k)>0 || s==true
            val2=[val2;subsetB(k)];
            s=true;
        end
    end
    % Generate synthetic data which mimics the 1-D location of a physical object moving at a constant speed.
    arr=['A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M' 'N' 'O' 'P' 'Q' 'R' 'S' 'T' 'U' 'V' 'W' 'X' 'Y' 'Z'];
    offset=1;
    val2=num2cell(val2);
    detectedLocations= val2;
    for idx = 1: length(detectedLocations)
        if detectedLocations{idx} == 0
            detectedLocations{idx} = [];
        end
    end
    %%
%     axes(handles.axes2);
    hold on;
    ylabel('Location');
    xlabel('Time');
    %%
    % Create a 1-D, constant speed Kalman filter when the physical object is first detected. Predict the location of the object based on previous states. If the object is detected at the current time step, use its location to correct the states.
    kalman = [];
    for idx = 1: length(detectedLocations)
        location = detectedLocations{idx};
        if isempty(kalman)
            if ~isempty(location)
                stateModel = [1 1;0 1];
                measurementModel = [1 0];
                kalman = vision.KalmanFilter(stateModel,measurementModel);
                kalman.State = [location, 0];
            end
        else
            trackedLocation = predict(kalman);
            if ~isempty(location)
                plot(idx, location,'k+');
                d = distance(kalman,location);
                title(sprintf('Distance:%f', d));
                trackedLocation = correct(kalman,location);
            else
                       title('Missing detection');
            end
            %       rang=strcat(arr(offset),num2str(i));
            %                     xlswrite('C:\Users\Hamza Zareen\Desktop\federerklmy.xlsx',trackedLocation,1,rang);
            %
            %                     offset=offset+1;
            pause(0.2);
            plot(idx,trackedLocation,'ro');
        end
    end
end

% --- Executes on button press in StartPrediction.
function StartPrediction_Callback(hObject, eventdata, handles)
% hObject    handle to FrameCalculation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes2);
prompt={'How many trajectories Prediction Results You Want To see'};
dlg_title='input';
num_line=1;
answer=inputdlg(prompt,dlg_title,num_line);
val=str2double(answer(1));
axes(handles.axes2);
if get(handles.PredictionFor2D, 'Value') == 1
    PredictionFor2DPrediction_Callback(val);
elseif get(handles.Predictionx, 'Value') == 1
    PredictionX_Callback(val);
elseif get(handles.Predictiony, 'Value') == 1
    PredictionY_Callback(val);
end
