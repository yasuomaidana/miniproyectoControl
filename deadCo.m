function [uk,ek] = deadCo(ua,err,set,lim)
    uk(:,1)=err(:,end-2)+ua(:,end-2);
    Sat=saturation('LinearInterval',[-lim,lim]); 
    uk=evaluate(Sat,uk);
    ek = set.'-uk; 
end