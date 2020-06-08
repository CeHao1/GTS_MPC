function y=smooth_overlap(x,k)
l=length(x);
[r,c]=size(x);
if r==1
    x1=repmat(x,1,3);
elseif c==1
    x1=repmat(x,3,1);
end

x2=smooth(x1,k);
y=x2(l+1:2*l);

end