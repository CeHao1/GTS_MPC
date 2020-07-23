function b1=tanh_fit2(b0,x,y)

myfunc = @(b1,x) b1(1)*tanh(b1(2)*x);
b1=nlinfit(x,y,myfunc,b0);
end