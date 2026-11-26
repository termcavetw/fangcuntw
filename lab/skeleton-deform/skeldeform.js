/* ==========================================================================
   v258 骨架變形（間架軸）
   方寸原本的軸只會改筆畫外面那一層（粗細、端點、質感），每個字的骨架只有一副，
   所以怎麼調都像同一套字。這一段讓骨架本身可以動：每一段筆畫給一個目標方向與
   長度，在「接點不散、T 接留在對方線段上、原本相交的交點比例不變」的限制下，
   用最小平方法解出所有控制點。參考實作：lab/deform2.py（逐點對照過）。

   ★ 只取「參數造成的差」：位置＝原位置＋(解－零參數解)。零參數時變形器本身會
     合併相近端點、把 T 接吸到線段上 —— 那不屬於風格，扣掉後原樣才是原樣。
   ★ 六根軸全部為 0 時直接回傳原陣列，輸出逐點不變（回歸測試會驗）。
   ★ 交錯保護：原本不相連的兩筆，變形後墨碰在一起（中心線距離小於筆寬），或
     相連的兩筆在接點處被壓扁到疊在一起，就先把那幾筆的變形量減半（3 輪），
     還不行再把整個字一起縮小，二分法找不撞的最大幅度。
   ★ 撇捺之所以是撇捺，就是因為它們是斜的：斜筆幾何化只把 25°–65° 的斜筆吸到
     45°，不吸成橫或豎（禾會變未、火會變兩豎）。只有很陡的撇照 skvsnap 拉直。
   ========================================================================== */
const SKD_PARAMS=["skspread","skext","skocto","skvsnap","skdotv","skeven"];
const SKD_TOL=14, SKD_NEAR=30, SKD_EG_OVERLAP=0.8, SKD_EG_W=2.0;
const SKD_CACHE=new Map();
function skdActive(f){ for(const k of SKD_PARAMS) if(f&&(+f[k]||0)!==0) return true; return false; }
function skdCls(v){
  const a=Math.atan2(Math.abs(v[1]),Math.abs(v[0]))*180/Math.PI;
  if(a<25) return "H";
  if(a>65) return "V";
  const d=v[1]<0?v:[-v[0],-v[1]];
  return d[0]<0?"P":"N";
}
function skdRot(v,deg){ const c=Math.cos(deg*Math.PI/180),s=Math.sin(deg*Math.PI/180); return [c*v[0]-s*v[1],s*v[0]+c*v[1]]; }
function skdCross(A,B,C,D,lo,hi){
  const r=[B[0]-A[0],B[1]-A[1]],s=[D[0]-C[0],D[1]-C[1]],den=r[0]*s[1]-r[1]*s[0];
  if(Math.abs(den)<1e-9) return null;
  const ca=[C[0]-A[0],C[1]-A[1]];
  const u=(ca[0]*s[1]-ca[1]*s[0])/den, w=(ca[0]*r[1]-ca[1]*r[0])/den;
  return (lo<u&&u<hi&&lo<w&&w<hi)?[u,w]:null;
}
function skdPD(P,A,B){
  const ab=[B[0]-A[0],B[1]-A[1]],L2=ab[0]*ab[0]+ab[1]*ab[1];
  const u=L2<1e-12?0:Math.min(1,Math.max(0,((P[0]-A[0])*ab[0]+(P[1]-A[1])*ab[1])/L2));
  return Math.hypot(A[0]+u*ab[0]-P[0],A[1]+u*ab[1]-P[1]);
}
function skdSegDist(A,B,C,D){
  if(skdCross(A,B,C,D,0,1)) return 0;
  return Math.min(skdPD(A,C,D),skdPD(B,C,D),skdPD(C,A,B),skdPD(D,A,B));
}
function skdAngle(u,v,line){
  const nu=Math.hypot(u[0],u[1]),nv=Math.hypot(v[0],v[1]);
  if(nu<1e-6||nv<1e-6) return 180;
  let a=Math.acos(Math.max(-1,Math.min(1,(u[0]*v[0]+u[1]*v[1])/(nu*nv))))*180/Math.PI;
  return line?Math.min(a,180-a):a;
}
/* 點到折線的最近距離（帶正負號＝在哪一側）；inner＝最近點不在折線兩端之外。 */
function skdNear(P,poly){
  let best={d:1e9,inner:false},bd=1e18;
  for(let i=0;i+1<poly.length;i++){
    const A=poly[i],B=poly[i+1],ab=[B[0]-A[0],B[1]-A[1]],L2=ab[0]*ab[0]+ab[1]*ab[1]; if(L2<1e-9) continue;
    const ap=[P[0]-A[0],P[1]-A[1]],u=(ap[0]*ab[0]+ap[1]*ab[1])/L2,uc=Math.max(0,Math.min(1,u));
    const dx=A[0]+uc*ab[0]-P[0],dy=A[1]+uc*ab[1]-P[1],dd=dx*dx+dy*dy;
    if(dd<bd){bd=dd;const sg=(ab[0]*ap[1]-ab[1]*ap[0])>=0?1:-1;
      best={d:sg*Math.sqrt(dd),inner:!((i===0&&u<0)||(i===poly.length-2&&u>1))};}
  }
  return best;
}
/* 拆成頂點與線段，找出接點、T 接、交叉、接點夾角。只算一次。 */
function skdSkeleton(strokes){
  const S={strokes,pts:[],idx:new Map()};
  strokes.forEach((st,si)=>st.p.forEach((p,pi)=>{S.idx.set(si+","+pi,S.pts.length);S.pts.push({si,pi,p:[p[0],p[1]]});}));
  const n=S.pts.length,par=[...Array(n).keys()];
  const find=i=>{while(par[i]!==i){par[i]=par[par[i]];i=par[i];}return i;};
  for(let i=0;i<n;i++)for(let j=0;j<i;j++){
    const a=S.pts[i],b=S.pts[j];
    if(a.si!==b.si&&Math.hypot(a.p[0]-b.p[0],a.p[1]-b.p[1])<SKD_TOL) par[find(i)]=find(j);
  }
  const roots=[...new Set([...Array(n).keys()].map(find))].sort((a,b)=>a-b);
  const vid=new Map(roots.map((r,k)=>[r,k]));
  S.var=[...Array(n).keys()].map(i=>vid.get(find(i)));
  S.nv=roots.length;
  S.P0=Array.from({length:S.nv},()=>[0,0]); const cnt=new Array(S.nv).fill(0);
  S.pts.forEach((q,k)=>{S.P0[S.var[k]][0]+=q.p[0];S.P0[S.var[k]][1]+=q.p[1];cnt[S.var[k]]++;});
  S.P0.forEach((q,k)=>{q[0]/=cnt[k];q[1]/=cnt[k];});
  let lx=1e9,ly=1e9,hx=-1e9,hy=-1e9;
  for(const q of S.pts){lx=Math.min(lx,q.p[0]);ly=Math.min(ly,q.p[1]);hx=Math.max(hx,q.p[0]);hy=Math.max(hy,q.p[1]);}
  S.lo=[lx,ly];S.hi=[hx,hy];S.c0=[(lx+hx)/2,(ly+hy)/2];S.hw=Math.max(1,(hx-lx)/2);
  S.segs=si=>{const o=[];for(let pi=0;pi+1<strokes[si].p.length;pi++)o.push([S.idx.get(si+","+pi),S.idx.get(si+","+(pi+1))]);return o;};
  S.edges=[];
  strokes.forEach((st,si)=>{for(const [a,b] of S.segs(si)){
    const v=[S.pts[b].p[0]-S.pts[a].p[0],S.pts[b].p[1]-S.pts[a].p[1]];
    if(Math.hypot(v[0],v[1])>1e-6) S.edges.push({va:S.var[a],vb:S.var[b],si,c:skdCls(v),v,
      m:[(S.pts[a].p[0]+S.pts[b].p[0])/2,(S.pts[a].p[1]+S.pts[b].p[1])/2]});
  }});
  const hl=new Map();
  for(const e of S.edges) if(e.c==="H") hl.set(e.si,(hl.get(e.si)||0)+Math.hypot(e.v[0],e.v[1]));
  S.main=null;
  if(hl.size>=2){let best=-1;for(const [k,v] of hl) if(v>best){best=v;S.main=k;}}
  const rel=new Set(),key=(i,j)=>Math.min(i,j)+","+Math.max(i,j);
  const sv=strokes.map(()=>new Set());
  S.pts.forEach((q,k)=>sv[q.si].add(S.var[k]));
  for(let i=0;i<strokes.length;i++)for(let j=0;j<i;j++){for(const x of sv[i]) if(sv[j].has(x)){rel.add(key(i,j));break;}}
  S.tjoints=[];
  S.pts.forEach((q,k)=>{
    for(let sj=0;sj<strokes.length;sj++){ if(sj===q.si) continue;
      for(const [a,b] of S.segs(sj)){
        if(S.var[a]===S.var[k]||S.var[b]===S.var[k]) continue;
        const A=S.pts[a].p,B=S.pts[b].p,ab=[B[0]-A[0],B[1]-A[1]],L2=ab[0]*ab[0]+ab[1]*ab[1];
        if(L2<1e-6) continue;
        const u=((q.p[0]-A[0])*ab[0]+(q.p[1]-A[1])*ab[1])/L2;
        if(.04<u&&u<.96&&Math.hypot(A[0]+u*ab[0]-q.p[0],A[1]+u*ab[1]-q.p[1])<SKD_TOL){
          S.tjoints.push([S.var[k],S.var[a],S.var[b],u]); rel.add(key(q.si,sj));
        }
      }
    }
  });
  S.crosses=[];
  for(let si=0;si<strokes.length;si++)for(let sj=0;sj<si;sj++)
    for(const [a,b] of S.segs(si))for(const [c,d] of S.segs(sj)){
      const r=skdCross(S.pts[a].p,S.pts[b].p,S.pts[c].p,S.pts[d].p,.06,.94);
      if(r){S.crosses.push([S.var[a],S.var[b],S.var[c],S.var[d],r[0],r[1]]);rel.add(key(si,sj));}
    }
  /* 相連兩筆在接點處的夾角 */
  const Q0=S.pts.map(q=>q.p), inc=new Map();
  strokes.forEach((st,si)=>{for(const [a,b] of S.segs(si)){
    if(!inc.has(S.var[a]))inc.set(S.var[a],[]); inc.get(S.var[a]).push([si,a,b]);
    if(!inc.has(S.var[b]))inc.set(S.var[b],[]); inc.get(S.var[b]).push([si,b,a]);
  }});
  const sub=(P,a,b)=>[P[b][0]-P[a][0],P[b][1]-P[a][1]];
  S.folds=[];
  for(const lst of inc.values())for(let x=0;x<lst.length;x++)for(let y=0;y<x;y++){
    const [si,a,b]=lst[x],[sj,c,d]=lst[y]; if(si===sj) continue;
    const ang=skdAngle(sub(Q0,a,b),sub(Q0,c,d),false);
    if(ang>=25) S.folds.push(["v",si,a,b,sj,c,d,ang]);
  }
  for(const [vk,va,vb] of S.tjoints)for(const [si,a,b] of (inc.get(vk)||[]))
    for(let sj=0;sj<strokes.length;sj++){ if(sj===si) continue;
      for(const [c,d] of S.segs(sj)){
        const vc=S.var[c],vd=S.var[d];
        if((vc===va&&vd===vb)||(vc===vb&&vd===va)){
          const ang=skdAngle(sub(Q0,a,b),sub(Q0,c,d),true);
          if(ang>=25) S.folds.push(["t",si,a,b,sj,c,d,ang]);
        }
      }
    }
  S.pairDist=(Q,i,j)=>{let best=1e9;for(const [a,b] of S.segs(i))for(const [c,d] of S.segs(j))best=Math.min(best,skdSegDist(Q[a],Q[b],Q[c],Q[d]));return best;};
  S.pairCross=(Q,i,j)=>{for(const [a,b] of S.segs(i))for(const [c,d] of S.segs(j))if(skdCross(Q[a],Q[b],Q[c],Q[d],.02,.98))return true;return false;};
  S.d0=[];
  for(let i=0;i<strokes.length;i++)for(let j=0;j<i;j++){
    if(rel.has(key(i,j))) continue;
    const d=S.pairDist(Q0,j,i);
    if(d<SKD_NEAR){rel.add(key(i,j));continue;}
    S.d0.push([j,i,d]);
  }
  S.rel=[...rel].map(k=>k.split(",").map(Number));
  /* 筆端靠近別筆時，記下它在那一筆「畫出來的形狀」的哪一側（彎筆用取樣後的弧，
     不是控制點連線）。變形後換邊＝筆端穿出去：火的點穿過拉直的捺就是這樣抓到的。 */
  S.vis0=strokes.map(st=>(Array.isArray(st.vis)&&st.vis.length>1)?st.vis:st.p);
  S.ends=[];
  S.pts.forEach((q,k)=>{
    if(q.pi!==0&&q.pi!==strokes[q.si].p.length-1) return;
    for(let sj=0;sj<strokes.length;sj++){ if(sj===q.si) continue;
      if(S.segs(sj).some(([a,b])=>S.var[a]===S.var[k]||S.var[b]===S.var[k])) continue;
      const r=skdNear(q.p,S.vis0[sj]);
      if(r.inner&&Math.abs(r.d)>3&&Math.abs(r.d)<120) S.ends.push([k,q.si,sj,r.d]);
    }
  });
  S.rel0={};
  return S;
}
/* 兩筆真的穿過對方：交點離兩條線段的端點都超過 m（墨的一半厚），端點貼著不算。 */
function skdPoke(S,Q,i,j,m){
  for(const [a,b] of S.segs(i))for(const [c,d] of S.segs(j)){
    const A=Q[a],B=Q[b],C=Q[c],D=Q[d],r=[B[0]-A[0],B[1]-A[1]],s=[D[0]-C[0],D[1]-C[1]];
    const den=r[0]*s[1]-r[1]*s[0]; if(Math.abs(den)<1e-9) continue;
    const ca=[C[0]-A[0],C[1]-A[1]],u=(ca[0]*s[1]-ca[1]*s[0])/den,w=(ca[0]*r[1]-ca[1]*r[0])/den;
    if(!(0<u&&u<1&&0<w&&w<1)) continue;
    if(Math.min(u,1-u)*Math.hypot(r[0],r[1])>m&&Math.min(w,1-w)*Math.hypot(s[0],s[1])>m) return true;
  }
  return false;
}
/* 最小平方：累加法方程 (AᵀA)X=AᵀB，再用 Cholesky 解（x、y 兩欄一起）。 */
function skdSolve(S,prm,alpha){
  const n=S.nv,N=new Float64Array(n*n),R=new Float64Array(n*2);
  const add=(coef,b,w)=>{const w2=w*w;
    for(const [i,ci] of coef){R[i*2]+=w2*ci*b[0];R[i*2+1]+=w2*ci*b[1];
      for(const [j,cj] of coef) N[i*n+j]+=w2*ci*cj;}};
  const spread=+prm.skspread||0,ext=+prm.skext||0,octo=+prm.skocto||0,vsnap=+prm.skvsnap||0,
        dotv=+prm.skdotv||0,evengap=+prm.skeven||0;
  for(const e of S.edges){
    const v=e.v,stt=S.strokes[e.si].t;let t=[v[0],v[1]];
    if(dotv&&stt==="dian"){
      const L=Math.hypot(v[0],v[1]),tv=[0,v[1]<=0?-L:L];
      t=[v[0]+(tv[0]-v[0])*dotv,v[1]+(tv[1]-v[1])*dotv];
    }else if((octo||vsnap)&&(stt==="pie"||stt==="na"||stt==="dian"||stt==="ti")){
      const a=Math.atan2(v[1],v[0])*180/Math.PI;
      let h=Math.abs(a)%180; h=Math.min(h,180-h);
      let ax=a,k=octo;
      if(h>=25&&h<=65) ax=Math.abs(a)<=90?(a<0?-45:45):(a<0?-135:135);
      /* 陡撇拉直獨立生效：不必先開斜筆幾何化 */
      if(vsnap&&stt==="pie"&&Math.abs(Math.abs(a)-90)<vsnap){ax=a>0?90:-90;k=1;}
      t=skdRot(v,(ax-a)*k);
    }else if(e.c==="H"){
      if(S.main!==null){const k=e.si===S.main?(1+ext):(1-ext/3);t=[t[0]*k,t[1]*k];}
    }else if(e.c==="V"){
      /* 豎畫內收（conv）在 Python 版有，實測太弱，app 不開放 */
    }else{
      const d=v[1]<0?v:[-v[0],-v[1]];
      t=skdRot(t,d[0]<0?-spread:spread);
    }
    const al=alpha[e.si];
    t=[v[0]+(t[0]-v[0])*al,v[1]+(t[1]-v[1])*al];
    add([[e.vb,1],[e.va,-1]],t,1);
  }
  for(const [vk,va,vb,u] of S.tjoints) add([[vk,1],[va,-(1-u)],[vb,-u]],[0,0],8);
  for(const [va,vb,vc,vd,u,w] of S.crosses) add([[va,1-u],[vb,u],[vc,-(1-w)],[vd,-w]],[0,0],8);
  if(evengap){
    const P0=S.P0;
    for(const [cl,axis] of [["H",1],["V",0]]){
      const sg=S.edges.filter(e=>e.c===cl&&Math.hypot(e.v[0],e.v[1])>60).map(e=>{
        const m=[(P0[e.va][0]+P0[e.vb][0])/2,(P0[e.va][1]+P0[e.vb][1])/2];
        return {va:e.va,vb:e.vb,m,lo:Math.min(P0[e.va][1-axis],P0[e.vb][1-axis]),hi:Math.max(P0[e.va][1-axis],P0[e.vb][1-axis])};
      });
      sg.sort((x,y)=>x.m[axis]-y.m[axis]);
      const used=new Array(sg.length).fill(false);
      for(let i=0;i<sg.length;i++){ if(used[i]) continue;
        const chain=[i];
        for(let j=i+1;j<sg.length;j++){
          const a=sg[chain[chain.length-1]],b=sg[j],ov=Math.min(a.hi,b.hi)-Math.max(a.lo,b.lo);
          const gap=b.m[axis]-a.m[axis];
          if(ov>SKD_EG_OVERLAP*Math.min(a.hi-a.lo,b.hi-b.lo)&&gap>30&&gap<260) chain.push(j);
        }
        if(chain.length<3) continue;
        for(const k of chain) used[k]=true;
        const g=(sg[chain[chain.length-1]].m[axis]-sg[chain[0]].m[axis])/(chain.length-1);
        for(let q=0;q+1<chain.length;q++){
          const a=sg[chain[q]],b=sg[chain[q+1]],cur=b.m[axis]-a.m[axis],tgt=cur+(g-cur)*evengap;
          const vv=[0,0]; vv[axis]=tgt; vv[1-axis]=b.m[1-axis]-a.m[1-axis];
          add([[b.va,.5],[b.vb,.5],[a.va,-.5],[a.vb,-.5]],vv,SKD_EG_W);
        }
      }
    }
  }
  for(let k=0;k<n;k++) add([[k,1]],S.P0[k],0.03);
  /* Cholesky（N 對稱正定；錨點項保證正定） */
  const L=new Float64Array(n*n);
  for(let i=0;i<n;i++)for(let j=0;j<=i;j++){
    let s=N[i*n+j]; for(let k=0;k<j;k++) s-=L[i*n+k]*L[j*n+k];
    if(i===j) L[i*n+i]=Math.sqrt(Math.max(s,1e-12)); else L[i*n+j]=s/L[j*n+j];
  }
  const X=Array.from({length:n},()=>[0,0]);
  for(const c of [0,1]){
    const y=new Float64Array(n);
    for(let i=0;i<n;i++){let s=R[i*2+c];for(let k=0;k<i;k++) s-=L[i*n+k]*y[k];y[i]=s/L[i*n+i];}
    for(let i=n-1;i>=0;i--){let s=y[i];for(let k=i+1;k<n;k++) s-=L[k*n+i]*X[k][c];X[i][c]=s/L[i*n+i];}
  }
  /* 放回字面框（主筆伸展時以主筆以外的範圍為準，主筆可以伸出去） */
  const keep=new Array(n).fill(true),olMask=new Array(S.pts.length).fill(true);
  if(S.main!==null&&ext>0){
    S.pts.forEach((q,k)=>{if(q.si===S.main){keep[S.var[k]]=false;olMask[k]=false;}});
    if(keep.filter(Boolean).length<2){keep.fill(true);olMask.fill(true);}
  }
  let ox0=1e9,oy0=1e9,ox1=-1e9,oy1=-1e9,nx0=1e9,ny0=1e9,nx1=-1e9,ny1=-1e9;
  S.pts.forEach((q,k)=>{if(!olMask[k])return;ox0=Math.min(ox0,q.p[0]);oy0=Math.min(oy0,q.p[1]);ox1=Math.max(ox1,q.p[0]);oy1=Math.max(oy1,q.p[1]);});
  X.forEach((q,k)=>{if(!keep[k])return;nx0=Math.min(nx0,q[0]);ny0=Math.min(ny0,q[1]);nx1=Math.max(nx1,q[0]);ny1=Math.max(ny1,q[1]);});
  const s=Math.min((ox1-ox0)/Math.max(1,nx1-nx0),(oy1-oy0)/Math.max(1,ny1-ny0));
  const nc=[(nx0+nx1)/2,(ny0+ny1)/2],oc=[(ox0+ox1)/2,(oy0+oy1)/2];
  for(const q of X){q[0]=(q[0]-nc[0])*s+oc[0];q[1]=(q[1]-nc[1])*s+oc[1];}
  let fx0=1e9,fy0=1e9,fx1=-1e9,fy1=-1e9;
  for(const q of X){fx0=Math.min(fx0,q[0]);fy0=Math.min(fy0,q[1]);fx1=Math.max(fx1,q[0]);fy1=Math.max(fy1,q[1]);}
  const fit=Math.min(1,960/Math.max(1,fx1-fx0),960/Math.max(1,fy1-fy0));
  if(fit<1){const fc=[(fx0+fx1)/2,(fy0+fy1)/2];for(const q of X){q[0]=(q[0]-fc[0])*fit+fc[0];q[1]=(q[1]-fc[1])*fit+fc[1];}}
  fx0=1e9;fy0=1e9;fx1=-1e9;fy1=-1e9;
  for(const q of X){fx0=Math.min(fx0,q[0]);fy0=Math.min(fy0,q[1]);fx1=Math.max(fx1,q[0]);fy1=Math.max(fy1,q[1]);}
  const dx=Math.max(0,20-fx0)-Math.max(0,fx1-980),dy=Math.max(0,20-fy0)-Math.max(0,fy1-980);
  for(const q of X){q[0]+=dx;q[1]+=dy;}
  return X;
}
function skdPositions(S,prm,alpha,Xb){
  const X=skdSolve(S,prm,alpha);
  return S.pts.map((q,k)=>[q.p[0]+X[S.var[k]][0]-Xb[S.var[k]][0],q.p[1]+X[S.var[k]][1]-Xb[S.var[k]][1]]);
}
function skdViolations(S,Q,clear){
  const bad=[];
  for(const [kind,si,a,b,sj,c,d,ang0] of S.folds){
    const ang=skdAngle([Q[b][0]-Q[a][0],Q[b][1]-Q[a][1]],[Q[d][0]-Q[c][0],Q[d][1]-Q[c][1]],kind==="t");
    if(ang<12&&ang<0.5*ang0) bad.push([Math.min(si,sj),Math.max(si,sj)]);
  }
  const vis=j=>{const poly=S.segs(j).map(([a])=>Q[a]).concat([Q[S.segs(j).slice(-1)[0][1]]]);
    return S.sampler?S.sampler(j,poly,!S.straight||S.straight[j]):poly;};
  const vc=new Map();
  for(const [k,si,sj,s0] of S.ends){
    if(Math.abs(s0)>=clear) continue;
    if(!vc.has(sj)) vc.set(sj,vis(sj));
    const r=skdNear(Q[k],vc.get(sj));
    if(r.inner&&r.d*s0<0&&Math.abs(r.d)>0.06*clear) bad.push([Math.min(si,sj),Math.max(si,sj)]);
  }
  /* 相連／相貼的筆：原本沒互相穿過，變形後穿過去（火、以、夂、辶 的長撇拉直後切過鄰筆） */
  if(!S.rel0[clear]){const Q0=S.pts.map(q=>q.p);S.rel0[clear]=S.rel.filter(([i,j])=>!skdPoke(S,Q0,i,j,.5*clear));}
  for(const [i,j] of S.rel0[clear]) if(skdPoke(S,Q,i,j,.5*clear)) bad.push([i,j]);
  for(const [i,j,d0] of S.d0){
    if(S.pairCross(Q,i,j)){bad.push([i,j]);continue;}
    const d1=S.pairDist(Q,i,j);
    if((d0>=clear&&d1<clear)||(d0<clear&&d1<0.7*d0)) bad.push([i,j]);
  }
  return bad;
}
/* 主入口：strokes=[{t,p}]，回傳每一筆的新控制點與保留幅度。 */
function skelDeform(strokes,prm,clear,report,sampler){
  const S=skdSkeleton(strokes); S.sampler=sampler||null;
  /* straight[j]：這一筆要不要跟著「斜筆幾何化」把弧拉直。
     拉直本身就可能讓別筆穿過來（火：點停在捺的弧上，捺一拉直點就穿過去），
     而那跟控制點怎麼動無關 —— 縮變形量救不了。所以撞到時先讓那兩筆保留原弧，
     還撞才減弱控制點的變形。 */
  S.straight=strokes.map(st=>true);
  const curved=strokes.map(st=>!!st.curved);
  const alpha=new Array(strokes.length).fill(1);
  const Xb=skdSolve(S,{},alpha);
  let Q=skdPositions(S,prm,alpha,Xb),lam=1;
  for(let r=1;r<=4;r++){
    const bad=skdViolations(S,Q,clear); if(!bad.length) break;
    let freed=false;
    for(const [i,j] of bad) for(const k of [i,j]) if(S.straight[k]&&curved[k]){S.straight[k]=false;freed=true;}
    if(freed) continue;
    for(const [i,j] of bad){alpha[i]*=.5;alpha[j]*=.5;}
    Q=skdPositions(S,prm,alpha,Xb);
  }
  if(skdViolations(S,Q,clear).length){
    let lo=0,hi=1,Qlo=S.pts.map(q=>q.p);
    for(let it=0;it<6;it++){
      const mid=(lo+hi)/2,pm={...prm,skeven:(+prm.skeven||0)*mid};
      const Qm=skdPositions(S,pm,alpha.map(a=>a*mid),Xb);
      if(skdViolations(S,Qm,clear).length) hi=mid; else {lo=mid;Qlo=Qm;}
    }
    lam=lo;Q=Qlo;
  }
  if(report){report.lam=lam;report.softened=alpha.filter(a=>a<1).length;report.kept=S.straight.filter(x=>!x).length;}
  const out=strokes.map((st,si)=>st.p.map((_,pi)=>Q[S.idx.get(si+","+pi)]));
  out.straight=S.straight.slice();
  return out;
}
/* 接到 collectRenderStrokes：只動 KAGE 骨架。斜筆幾何化同時把弧線拉直。 */
function skelDeformApply(raw,f,id,variant){
  if(!skdActive(f)||!raw||raw.length<1) return raw;
  const {hw,sw}=widths(f), clear=(hw+sw)/2;
  const key=id+"|"+(variant||"")+"|"+(+f.pratio||0).toFixed(3)+"|"+SKD_PARAMS.map(k=>(+f[k]||0).toFixed(3)).join(",")+"|"+clear.toFixed(1);
  let P=SKD_CACHE.get(key);
  if(!P){
    const oc=Math.max(0,Math.min(1,+f.skocto||0));
    const smp=(s,pts,b)=>sampleSkeleton(pts,b,0,16,oc>=.5&&pts!==s.p?null:s.cv,s.kj,oc>=.5&&pts!==s.p?null:s.qc).map(q=>q.p);
    const crv=s=>Math.abs(s.bend||0)>.02||!!s.cv||!!s.qc;
    try{ P=skelDeform(raw.map(s=>({t:s.t,p:s.p,vis:smp(s,s.p,s.bend||0),curved:oc>0&&crv(s)})),f,clear,null,
                      (j,poly,str)=>!str?sampleSkeleton(poly,raw[j].bend||0,0,16,raw[j].cv,raw[j].kj,raw[j].qc).map(q=>q.p)
                                    :(oc>=.5?poly:smp(raw[j],poly,(raw[j].bend||0)*(1-oc)))); }
    catch(e){ return raw; }                      /* 變形失敗一律退回原骨架 */
    if(SKD_CACHE.size>6000) SKD_CACHE.clear();
    SKD_CACHE.set(key,P);
  }
  const octo=Math.max(0,Math.min(1,+f.skocto||0));
  return raw.map((s,i)=>{
    const o={...s,p:P[i].map(q=>[q[0],q[1]])};
    if(octo>0&&(!P.straight||P.straight[i])){ o.bend=(s.bend||0)*(1-octo); if(octo>=.5){o.cv=null;o.qc=null;} }
    return o;
  });
}
