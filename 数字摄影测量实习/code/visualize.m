
clear;clc;
% img=imread('E:\C++\Digitaltest\data\DPEX_Data11\rock1.orl.bmp');
% b=size(img);
% RGB = insertShape(img,'circle',[500 500 100],'LineWidth',5);
% 
% imshow(RGB);
A={[1 2],[3 4],[5,6]};
B=A(1,2:3);



img=imread('E:\C++\Digitaltest\data\DPEX_Data11\rock1.orl.bmp');
RGB=img;
color=["y" "m" "c" "r" "g" "b"];
path1='E:\C++\Digitaltest\data\DPEX_Data09\lines';
name1=dir(path1);
count=0;
for i = 1 : length( name1 ) 
    if( isequal( name1( i ).name, '.' ) ||  isequal( name1( i ).name, '..' ) || ~name1( i ).isdir )   % 如果不是目录跳过 
       continue; 
    end 
    subdirpath = fullfile( path1, name1(i).name, '*.txt' ); 
    namelist=dir(subdirpath);
    l2=length(namelist);
    for j=1:l2
        path=fullfile(path1, name1(i).name,namelist(j).name);
        fileID = fopen(path,'r');
        d=fscanf(fileID,'%f',[3 Inf]);
        d=d';
        plot(d(:,1),-d(:,2),color(mod(i,6)+1));
        axis equal
        hold on
        
        count=count+1;
        cr=size(d,1);
        C=ones(cr-1,4);
        for k=1:cr-1
            C(k,1)=round(d(k,1)-1365.24);
            C(k,2)=round(d(k,2)-297.018);
            C(k,3)=round(d(k+1,1)-1365.24);
            C(k,4)=round(d(k+1,2)-297.018);
        end
        RGB = insertShape(RGB,'Line',C,'LineWidth',5);
    end
end
hold off
imwrite(RGB,'E:\C++\Digitaltest\data\DPEX_Data11\DEMphoto.bmp');
imshow(RGB);


