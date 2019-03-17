function getguess
a_0 = zeros(1,4);
a_e = [10,10,10,10];
u = 0.291;
for i =1 :100
   c = a_e.*P_pronk(a_0,u) - a_0.*P_pronk(a_e,u);
   
   if P_pronk(c,u) == 0 
       break
   end
   
   if P_pronk(a_0,u).*P_pronk(c,u) < 0 
       
    a_e = c ;
   else
       a_0 = c;
   end
   
    
    
    
    
end


end