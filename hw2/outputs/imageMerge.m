function imageMerge(image1, image2)

c = hgload(image1);
k = hgload(image2);

figure
h(1) = subplot(1,2,1); xlabel('Value of x'); ylabel('Function Value');
h(2) = subplot(1,2,2); xlabel('Value of Linearly Spaced x'); ylabel('Error Value');

copyobj(allchild(get(c,'CurrentAxes')),h(1));
copyobj(allchild(get(k,'CurrentAxes')),h(2));

end