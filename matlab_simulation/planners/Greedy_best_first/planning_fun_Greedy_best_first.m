function [final_path] = planning_fun_Greedy_best_first(state_robot,dt,limit,goal,image,resolution,maxIter)

grid_search = Greedy_best_first(state_robot,dt,limit,goal,image,resolution,maxIter);
finish = 0;
while(finish == 0)
   next_cell = grid_search.find_next_cell();
   if(next_cell == -1)
       finish = 1;
       break;
   end
   finish = grid_search.check_goal(next_cell);
   if(finish == 1)
       break;
   end
   grid_search.open_sons_cell(next_cell);
  
    %for test/plot
    rgbImage = cat(3, image, image, image);
    rgbImage(int16(goal(1)/resolution)+1,int16(goal(2)/resolution)+1,1) = 255;
    rgbImage(int16(goal(1)/resolution)+1,int16(goal(2)/resolution)+1,2) = 0;
    rgbImage(int16(goal(1)/resolution)+1,int16(goal(2)/resolution)+1,3) = 0;
    for i = 1:size(image,1)
       for j = 1:size(image,2) 
              if(grid_search.cells_isopen(i +(j-1)*size(image,2),1) == 1)
                  rgbImage(i+1,j+1,1) = 0;
                  rgbImage(i+1,j+1,2) = 255;
                  rgbImage(i+1,j+1,3) = 0;
              end
       end
    end
   J = imrotate(rgbImage,90); J = imresize( J , 5); imshow(J);
end
final_path = grid_search.take_path();
end

