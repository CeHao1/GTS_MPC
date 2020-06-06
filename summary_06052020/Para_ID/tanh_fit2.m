function b1=tanh_fit(b0,x,y)

myfunc1=inline('b1(1)*tanh(b1(2)*x)','b1','x'); 
b1=nlinfit(x,y,myfunc1,b0);


end