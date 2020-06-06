function x=extend(x)
% y=[x,x(end)*2-x(end-1)];
len=length(x);
x(len+1)=x(len)*2-x(len-1);

end