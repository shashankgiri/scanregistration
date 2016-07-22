function plane = planefit3D(ptCloud)
model = pcfitplane(ptCloud,0.01,[0,0,1],2);
plane1 = model.Normal';
if plane1(3,1) <0
    plane = -plane1;
else
    plane = plane1;
end
end