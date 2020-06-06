function [xCoeff,yCoeff,xTraj,yTraj,cost]=TrajOpt(WayPoints,Times,polyOrder)

wpConcat=WayPoints';
dim=size(wpConcat);
numWp=dim(2);
numTraj=numWp-1;
constDer=(polyOrder+1)/2;

p = sym('p', [polyOrder+1,1]);
syms t T

pol=0;
for i=1:length(p)
    pol=pol+p(i)*t^(length(p)-i);
end
snap=diff(pol,t,4);
cost=int(snap^2,t,0,T);
Qi=(1/2)*hessian(cost,p);
Q=cell(2*numTraj,2*numTraj);
for i=1:numTraj
    idx=2*(i-1)+1;
    Q{idx,idx}=eval(subs(Qi,T,Times(i)));
    Q{idx+1,idx+1}=eval(subs(Qi,T,Times(i)));
end
for i=1:length(Q)
    for n=1:length(Q)
        if isempty(Q{i,n})
            Q{i,n}=zeros(length(p));
        end
    end
end
Q=cell2mat(Q);






Traj=zeros(4,numTraj);
for i=1:numTraj
    if i~=1
        Traj(1,i)=0;
        Traj(2,i)=0;
        Traj(3,i)=wpConcat(1,i+1);
        Traj(4,i)=wpConcat(2,i+1);
    else
        Traj(1,i)=wpConcat(1,i);
        Traj(2,i)=wpConcat(2,i);
        Traj(3,i)=wpConcat(1,i+1);
        Traj(4,i)=wpConcat(2,i+1);
    end
end
dim=size(Traj);
pos=reshape(Traj,dim(1)*dim(2),1);
d=zeros(length(Q),1);
for i=1:length(pos)
    idx=(i-1)*constDer+1;
    d(idx)=pos(i);
end

dpInd=[];
i=2*constDer+2;
while i<length(d)-2*constDer
    for m=1:constDer-1
        dpInd(end+1)=i;
        i=i+1;
    end
    i=i+1;
    for m=1:constDer-1
        dpInd(end+1)=i;
        i=i+1;
    end
    i=i+1;
    i=i+2*constDer;
end

df=d; df(dpInd)=[];
for i=1:length(dpInd)
    d(dpInd(i))=99;
end


reorder=zeros(1,length(d)-length(dpInd));
i=1; count=1;
while i<=length(d)
    if sum(i==dpInd)==0
        reorder(count)=i;
        count=count+1;
    end
    i=i+1;
end
reorder=[reorder,dpInd];

Cinv_t=eye(length(d));
Cinv_t=Cinv_t(reorder,:);
C=inv(Cinv_t);




for i=1:constDer
    for n=1:length(p)
        Ai(i,n)=diff(t^(length(p)-n),t,i-1);
    end
end
A=cell(4*numTraj,2*numTraj);
for i=1:(numTraj)
    idx1=4*(i-1)+1;
    idx2=2*(i-1)+1;
    A{idx1,idx2}=eval(subs(Ai,t,0));
    A{idx1+1,idx2+1}=eval(subs(Ai,t,0));
    A{idx1+2,idx2}=eval(subs(Ai,t,Times(i)));
    A{idx1+3,idx2+1}=eval(subs(Ai,t,Times(i)));
    if i>1
        A{idx1,idx2-2}=(-1)*eval(subs(Ai,t,Times(i-1)));
        A{idx1+1,idx2-1}=(-1)*eval(subs(Ai,t,Times(i-1)));
    end
end
for i=1:4*numTraj
    for n=1:2*numTraj
        if isempty(A{i,n})
            A{i,n}=zeros(constDer,length(p));
        end
    end
end
A=cell2mat(A);




R=C'*inv(A)'*Q*inv(A)*C;
Rfp=R(1:length(df),length(df)+1:end);
Rpp=R(length(df)+1:end,length(df)+1:end);

dp=-inv(Rpp)*Rfp'*df;
dfdp=[df;dp];
d=C*dfdp;





coeff=inv(A)*d;

dt=0.01;
T=cell(1,numTraj);

coefMat=reshape(coeff,polyOrder+1,numTraj*2);
xCoeff=cell(1,numTraj);
yCoeff=cell(1,numTraj);
xTraj=cell(1,numTraj);
yTraj=cell(1,numTraj);

for i=1:numTraj
    T{i}=0:dt:Times(i);
    x=zeros(1,length(T{i}));
    y=zeros(1,length(T{i}));
    idx=2*(i-1)+1;
    xCoeff{i}=coefMat(:,idx);
    yCoeff{i}=coefMat(:,idx+1);
    for n=1:length(T{i})
        x(n)=0;
        y(n)=0;
        order=polyOrder;
        for j=1:polyOrder+1
            x(n)=x(n)+xCoeff{i}(j)*T{i}(n)^order;
            y(n)=y(n)+yCoeff{i}(j)*T{i}(n)^order;
            order=order-1;
        end
    end
    xTraj{i}=x;
    yTraj{i}=y;
end


cost=coeff'*Q*coeff;





end





