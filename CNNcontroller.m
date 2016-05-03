classdef CMACcontroller
   properties
      in_max;
      in_min;
      resolusion;
      c;
      weight;
      in_dimension;
      out_dimension;
      NAP;
      PrimeList=[ 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, ...
        43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 113,   ...
        193, 241, 257, 337, 353, 401, 433, 449, 577, 593, 641,   ...
        673, 769, 881, 929, 977, 1009, 1153, 1201, 1217, 1249,   ...
        1297,1361, 1409, 1489, 1553, 1601, 1697, 1777, 1873,   ...
        1889, 2017, 2081, 2113, 2129, 2161, 2273, 2417, 2593,   ...
        2609, 2657, 2689, 2753, 2801, 2833, 2897, 3041, 3089,   ...
        3121, 3137, 3169, 3217, 3313, 3329, 3361, 3457, 3617,   ...
        3697, 3761, 3793, 3889, 4001, 4049, 4129, 4177, 4241,   ...
        4273, 4289, 4337, 4481, 4513, 4561, 4657, 4673, 4721,   ...
        4801, 4817, 4993, 5009, 5153, 5233, 5281, 5297, 5393,   ...
        5441, 5521, 5569, 5857, 5953, 6113, 6257, 6337, 6353,   ...
        6449, 6481, 6529, 6577, 6673, 6689, 6737, 6833, 6961,   ...
        6977, 7057, 7121, 7297, 7393, 7457, 7489, 7537, 7649,   ...
        7681, 7793, 7841, 7873, 7937, 8017, 8081, 8161, 8209,   ...
        8273, 8353, 8369, 8513, 8609, 8641, 8689, 8737, 8753,   ...
        8849, 8929, 9041, 9137, 9281, 9377, 9473, 9521, 9601,   ...
        9649, 9697, 9857];
   end
   
   methods
       function obj=CMACcontroller(limit, c, out_dimension)%limit[[in_max in_min]'*in_dimension], resolution[1*in_dimension], c[1*1]
           obj.in_dimension=length(limit(1,:));
           obj.out_dimension=out_dimension;
           for i=1:obj.in_dimension
               obj.in_max(i)=limit(1,i);
               obj.in_min(i)=limit(2,i);
               obj.resolusion(i)=(obj.in_max(i)-obj.in_min(i))/(100*c);
           end
           %NAP=10*c+1;%取10c最近的质数
           NAP=FindNearestPrime(10*c, obj.PrimeList);
           obj.weight=WeightInit(obj.in_dimension, obj.out_dimension, NAP);
           obj.c=c;
           obj.NAP=NAP;
%            obj.weight=zeros(NAP,NAP,obj.out_dimension);
       end
       
       function [obj, out] = CMACrecaller(obj, in, outref, trainning_flag)
        alpha=0.5;
        M=zeros(obj.in_dimension,obj.c);
        for i=1:obj.in_dimension
        in_quant(i)=floor((in(i)-obj.in_min(i))/obj.resolusion(i));
        end
        for i=1:obj.c
            for j=1:obj.in_dimension
            M(j,mod((in_quant(j)+i),obj.c)+1)=in_quant(j)+i;
            end
        end
        out=zeros(1,obj.out_dimension);
        for j=1:obj.out_dimension
            for i=1:obj.c
                switch obj.in_dimension
                    case 1
                    out(j)=obj.weight(mod(M(1,i),obj.NAP)+1, j)+out(j);
                    case 2
                    out(j)=obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, j)+out(j);
                    case 3
                    out(j)=obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, j)+out(j);
                    case 4
                    out(j)=obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, j)+out(j);
                    case 5
                    out(j)=obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, j)+out(j);
                    case 6
                    out(j)=obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, mod(M(6,i),obj.NAP)+1, j)+out(j);
                    case 9
                    out(j)=obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, mod(M(6,i),obj.NAP)+1, mod(M(7,i),obj.NAP)+1, mod(M(8,i),obj.NAP)+1, mod(M(9,i),obj.NAP)+1, j)+out(j);    
                end
            end
        end
        if trainning_flag
        for j=1:obj.out_dimension
            trainning_error=outref(j)-out(j);
%             trainning_error
% %             switch j
% %                 case 1
% %                     deadzone=5*1e-4;
% %                 case 2
% %                     deadzone=5*1e-4;
% %                 case 3
% %                     deadzone=5*1e-4;
% %             end
% %             if abs(trainning_error)<deadzone
% %                 trainning_error=0;
% %             end
% %             if trainning_error>deadzone
% %                 trainning_error=trainning_error-deadzone;
% %             end
% %             if trainning_error<-deadzone
% %                 trainning_error=trainning_error+deadzone;
% %             end
%             trainning_error
            for i=1:obj.c
                switch obj.in_dimension
                    case 1
                        obj.weight(mod(M(1,i),obj.NAP)+1, j)=...
                        obj.weight(mod(M(1,i),obj.NAP)+1, j)+alpha*(trainning_error)/obj.c;
                    case 2
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, j)=...
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, j)+alpha*(trainning_error)/obj.c;
                    case 3
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, j)=...
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, j)+alpha*(trainning_error)/obj.c;
                    case 4
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, j)=...
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, j)+alpha*(trainning_error)/obj.c;
                    case 5
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, j)=...
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, j)+alpha*(trainning_error)/obj.c;
                    case 6
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, mod(M(6,i),obj.NAP)+1, j)=...
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, mod(M(6,i),obj.NAP)+1, j)+alpha*(trainning_error)/obj.c;
                    case 9
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, mod(M(6,i),obj.NAP)+1, mod(M(7,i),obj.NAP)+1, mod(M(8,i),obj.NAP)+1, mod(M(9,i),obj.NAP)+1, j)=...
                        obj.weight(mod(M(1,i),obj.NAP)+1, mod(M(2,i),obj.NAP)+1, mod(M(3,i),obj.NAP)+1, mod(M(4,i),obj.NAP)+1, mod(M(5,i),obj.NAP)+1, mod(M(6,i),obj.NAP)+1, mod(M(7,i),obj.NAP)+1, mod(M(8,i),obj.NAP)+1, mod(M(9,i),obj.NAP)+1, j)+alpha*(trainning_error)/obj.c;
                end
            end
        end
        %outref-out
        end
       end
       
   end
end

function weight=WeightInit(in_dimension, out_dimension, NAP)
switch in_dimension
    case 1
        weight=zeros(NAP, out_dimension);
    case 2
        weight=zeros(NAP, NAP, out_dimension);
    case 3
        weight=zeros(NAP, NAP, NAP, out_dimension);
    case 4
        weight=zeros(NAP, NAP, NAP, NAP, out_dimension);
    case 5
        weight=zeros(NAP, NAP, NAP, NAP, NAP, out_dimension);
    case 6
        weight=zeros(NAP, NAP, NAP, NAP, NAP, NAP, out_dimension);
    case 9
        weight=zeros(NAP, NAP, NAP, NAP, NAP, NAP, NAP, NAP, NAP, out_dimension);
end
end

function y=FindNearestPrime(x, PrimeList)
% while 1==1
%     if IsPrime(x, PrimeList)
%         y=x;
%         break
%     end
%     x=x+1;
% end
for i=1:length(PrimeList)
    if PrimeList(i)>x
        y=PrimeList(i);
        break
    end
end
end

function primeflag=IsPrime(x, PrimeList)
for i=1:length(PrimeList)
    if length(PrimeList)/2+1<i
        primeflag=true;
        break
    end
    if mod(x,PrimeList(i))==0
        primeflag=false;
        break
    end
end
end