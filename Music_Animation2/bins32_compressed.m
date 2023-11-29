clear all
pkg load instrument-control
pkg load image

s = serialport("COM3",115200); 
%fopen(s) ;
%
%I = imread("hunter.png");
%I = imread("bruce_tahmid.png");
I = imread("towers.png");
%I = imread("kids.png");
a = imresize(I,0.2);
%figure 1; clf
%imshow(a);
%
ymax = columns(a) 
xmax = rows(a) 

%frame start
pause(0.1) ;
fprintf(s,"%x\r", 256) ;
pause (0.1) ;
% set x position 1024 is set cmd
fprintf(s,"%x\r", 4*256 + 80) ;
%pause (0.1) ;
% set y posiion 8*256 is set cmd
fprintf(s,"%x\r", 8*256 + 120) ;
%pause (0.1) ;
%
for x=1:xmax
  for y=1:ymax
    if(y==1) 
      fprintf(s,"%x\r", 512) ;
    endif
 
    r = a(x,y,1);
    if(r<32)
      red = 1 ;
    elseif (r<64) %64
      red= 2 ;
    elseif(r<96)
      red = 2;
    elseif (r<128) 
      red = 3 ;
    elseif (r<160)  %160
      red= 4; 
    elseif(r<192)
      red = 5;
    elseif (r<224) %224
      red = 7;
    else
      red = 7 ;
    endif
      
    g = a(x,y,2);
    if(g<32)
      green = 1 ;
    elseif (g<64)
      green= 2 ;
    elseif(g<96)
      green = 2;
    elseif (g<128) %128
      green = 3 ;
    elseif (g<160) %160
      green= 4; 
    elseif(g<192) %192
      green = 5;
    elseif (g<224) %224
      green = 7;
    else
      green = 7 ;
    endif
    
    b = a(x,y,3);
    if(b<64)
      blue = 0 ;
    elseif (b<128)
      blue= 1 ;
    elseif(b<192)
      blue = 2;
    else
      blue = 3 ;
    endif
 
    draw_cmd = red*32 + green*4 + blue ;
    fprintf(s, "%x\r", draw_cmd) ;
  endfor
endfor
% frame end
fprintf(s,"%x\r", 3*256) ;

%fprintf("%d %d\n\r", xmax, ymax);
%fprintf("%d %d %d\n\r", max_red, max_green, max_blue);
% close the serial port
clear s

