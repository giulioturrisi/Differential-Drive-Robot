image = imread('coppeliasim_simple.pgm');
size_image = size(image)

for i = 1:size_image(1)
   for j = 1:size_image(2)
      if(image(i,j) < 250)
          image(i,j) = 0;
      end
   end

end

%inflate
image_copy = image;
resolution = 0.05;
radius = 0.15;
to_inflate = radius/resolution;
for i = 1:size_image(1)
   for j = 1:size_image(2)
      if(image_copy(i,j) < 250)
          for k = 1:to_inflate
              for d = 1:to_inflate
                  if(i-k > 0 & j-d > 0)
                    image(i-k,j-d) = 0;
                  end
                  if(i-k > 0 & j+d < size_image(2))
                    image(i-k,j+d) = 0;
                  end
                  if(i+k < size_image(1) & j-d > 0)
                    image(i+k,j-d) = 0;
                  end
                  if(i+k < size_image(1) & j+d < size_image(2))
                    image(i+k,j+d) = 0;
                  end
              end
          end
      end
   end

end

imwrite(image,'coppeliasim_simple_inflated2.pgm','pgm','Encoding','ASCII')