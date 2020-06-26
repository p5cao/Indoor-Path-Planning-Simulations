function Children = Neib(node, search_step, obstacles)

next = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
next_nodes = node + search_step*next;
Children = [];
for i = 1:length(next_nodes)
   if  CollisionFree(node, next_nodes(i,:), obstacles) 
       x = next_nodes(i,:);
       if  x(1)>= 0 && x(1) <= 700 && x(2)>=0 && x(2)<=320
            Children = [Children; next_nodes(i,:)];
       end
    end

end