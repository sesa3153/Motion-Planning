function [Matx,Maty,Matz] = CreateVertices(center,length,width,height)

%front face
x1 = (center(1)+length/2).*ones(1,4);
y1 = [ center(2)+width/2 center(2)+width/2 center(2)-width/2 center(2)-width/2];
z1 = [ center(3)+height/2 center(3)-height/2 center(3)-height/2 center(3)+height/2];

%back face
x2 =  (center(1) - length/2).*ones(1,4);
y2 = y1;
z2 = z1;

%right face
x3 = [center(1)-length/2 center(1)-length/2 center(1)+length/2 center(1)+length/2];
y3 = (center(2)+width/2).*ones(1,4);
z3 = z1;

%left face
x4 = x3;
y4 = (center(2)-width/2).*ones(1,4);
z4 = z1;

%top face
x5 = [center(1)+length/2 center(1)-length/2 center(1)-length/2 center(1)+length/2];
y5 = [center(2)+width/2 center(2)+width/2 center(2)-width/2 center(2)-width/2];
z5 = (center(3) + height/2).*ones(1,4);

%bottom face
x6 = x5;
y6 = y5;
z6 = (center(3) - height/2).*ones(1,4);

Matx = [x1;x2;x3;x4;x5;x6];
Maty = [y1;y2;y3;y4;y5;y6];
Matz = [z1;z2;z3;z4;z5;z6];

end

