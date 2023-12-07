#include <iostream>
#include <math.h>
using namespace std;


const int npts = 100;

struct Curves_struct{
    bool ok;
    float sc_s1;
    float sc_s2;
    float sc_s3;
};

struct Circle_line{
    float x;
    float y;
    float th;

    void print(){
        cout<<x<<"\n";
        cout<<y<<"\n";
        cout<<th<<"\n";
    }
};

struct Plot_arc_struct{
    float pts[npts+1][2];
};

struct Dubins_arc{
        float x0;
        float y0;
        float th0;
        float k;
        float L;
        float xf;
        float yf;
        float thf;
    void print(){
        cout<<"x0: "<<x0<<"\n";
        cout<<"y0: "<<y0<<"\n";
        cout<<"th0: "<<th0<<"\n";
        cout<<"k: "<<k<<"\n";
        cout<<"L: "<<L<<"\n";
        cout<<"xf: "<<xf<<"\n";
        cout<<"yf: "<<yf<<"\n";
        cout<<"thf: "<<thf<<"\n";
    }
};

struct Dubins_curve{
        Dubins_arc a1;
        Dubins_arc a2;
        Dubins_arc a3;
        float L;
};

struct Scale_to_standard{
    float sc_th0;
    float sc_thf;
    float sc_Kmax;
    float lambda;

    void print(){
        cout<<sc_th0<<"\n";
        cout<<sc_thf<<"\n";
        cout<<sc_Kmax<<"\n";
        cout<<lambda<<"\n";

    };
};

struct Scale_from_standard{
    float s1;
    float s2;
    float s3;
};


Dubins_curve dubins_curve(float x0, float y0, float th0, float s1, float s2, float s3, float k0, float k1, float k2);
Dubins_arc dubins_arc(float x0, float y0, float th0, float k, float L);
Scale_to_standard scale_to_standard(float x0, float y0, float th0, float xf, float yf, float thf, float Kmax);
Scale_from_standard scale_from_standard(float lambda, float sc_s1, float sc_s2, float sc_s3);
float mod2pi(float ang);
Dubins_curve dubins_shortest_path(float x0, float y0, float th0, float xf, float yf, float thf, float Kmax);
Curves_struct LSL(float sc_th0, float sc_thf, float sc_Kmax);
Curves_struct RSR(float sc_th0, float sc_thf, float sc_Kmax);
Curves_struct LSR(float sc_th0, float sc_thf, float sc_Kmax);
Curves_struct RSL(float sc_th0, float sc_thf, float sc_Kmax);
Curves_struct RLR(float sc_th0, float sc_thf, float sc_Kmax);
Curves_struct LRL(float sc_th0, float sc_thf, float sc_Kmax);
Circle_line circline(float s, float x0, float y0, float th0, float k);
void plot_dubins(Dubins_curve curve);



Dubins_curve dubins_shortest_path(float x0, float y0, float th0, float xf, float yf, float thf, float Kmax){
    float sc_th0, sc_thf, sc_Kmax, lambda, s1, s2, s3;
    
    Scale_to_standard s;
    s = scale_to_standard(x0, y0, th0, xf, yf, thf, Kmax);
    sc_th0 = s.sc_th0; sc_thf = s.sc_thf; sc_Kmax = s.sc_Kmax; lambda = s.lambda;

    int pidx = -1;
    int ksigns[6][3] = {{1,0,1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1}};
    float L = INFINITY;
    float sc_s1, sc_s2, sc_s3;

    for(int i = 0; i < 6; i++){
        bool ok;
        float Lcur;
        float sc_s1_c, sc_s2_c, sc_s3_c;
        switch(i){
            case 0:
                Curves_struct temp6;
                temp6 = LSL(sc_th0, sc_thf, sc_Kmax);
                ok = temp6.ok; sc_s1_c = temp6.sc_s1; sc_s2_c = temp6.sc_s2; sc_s3_c = temp6.sc_s3;
                Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
                if(ok && Lcur < L){
                    L = Lcur;
                    sc_s1 = sc_s1_c;
                    sc_s2 = sc_s2_c;
                    sc_s3 = sc_s3_c;
                    pidx = i;
                    
                }
                break;

            case 1:
                Curves_struct temp5;
                temp5 = RSR(sc_th0, sc_thf, sc_Kmax);
                ok = temp5.ok; sc_s1_c = temp5.sc_s1; sc_s2_c = temp5.sc_s2; sc_s3_c = temp5.sc_s3;
                Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
                if(ok && Lcur < L){
                    L = Lcur;
                    sc_s1 = sc_s1_c;
                    sc_s2 = sc_s2_c;
                    sc_s3 = sc_s3_c;
                    pidx = i;
                }
                break;
            
            case 2:
                Curves_struct temp4;
                temp4 = LSR(sc_th0, sc_thf, sc_Kmax);
                ok = temp4.ok; sc_s1_c = temp4.sc_s1; sc_s2_c = temp4.sc_s2; sc_s3_c = temp4.sc_s3;
                Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
                if(ok && Lcur < L){
                    L = Lcur;
                    sc_s1 = sc_s1_c;
                    sc_s2 = sc_s2_c;
                    sc_s3 = sc_s3_c;
                    pidx = i;
                }
                break;

            case 3:
                Curves_struct temp3;
                temp3 = RSL(sc_th0, sc_thf, sc_Kmax);
                ok = temp3.ok; sc_s1_c = temp3.sc_s1; sc_s2_c = temp3.sc_s2; sc_s3_c = temp3.sc_s3;
                Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
                if(ok && Lcur < L){
                    L = Lcur;
                    sc_s1 = sc_s1_c;
                    sc_s2 = sc_s2_c;
                    sc_s3 = sc_s3_c;
                    pidx = i;
                }
                break;
            case 4:
                Curves_struct temp2;
                temp2 = RLR(sc_th0, sc_thf, sc_Kmax);
                ok = temp2.ok; sc_s1_c = temp2.sc_s1; sc_s2_c = temp2.sc_s2; sc_s3_c = temp2.sc_s3;
                Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
                if(ok && Lcur < L){
                    L = Lcur;
                    sc_s1 = sc_s1_c;
                    sc_s2 = sc_s2_c;
                    sc_s3 = sc_s3_c;
                    pidx = i;
                }
                break;

            case 5:
                Curves_struct temp1;
                temp1 = LRL(sc_th0, sc_thf, sc_Kmax);
                ok = temp1.ok; sc_s1_c = temp1.sc_s1; sc_s2_c = temp1.sc_s2; sc_s3_c = temp1.sc_s3;
                Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
                if(ok && Lcur < L){
                    L = Lcur;
                    sc_s1 = sc_s1_c;
                    sc_s2 = sc_s2_c;
                    sc_s3 = sc_s3_c;
                    pidx = i;
                }
                break;
        }
    }
    Dubins_curve curve;
    if(pidx >= 0){
        Scale_from_standard temp;
        
        temp = scale_from_standard(lambda, sc_s1, sc_s2, sc_s3);
        s1 = temp.s1; s2 = temp.s2; s3 = temp.s3;
        

        curve = dubins_curve(x0, y0, th0, s1, s2, s3, ksigns[pidx][0]*Kmax, ksigns[pidx][1]*Kmax, ksigns[pidx][2]*Kmax);
        
        curve.a1.print();
        cout<<"\n";
        curve.a2.print();
        cout<<"\n";
        curve.a3.print();
        cout<<curve.L;

        //plot_dubins(curve);
        return curve;
    }

    return curve;
    
    
}

float sinc(float t){
    float s;
    if(abs(t)<0.002){
        s = 1 - pow(t,(2/6)) * (1 - pow(t,2/20));
        
    }else{
        s = sin(t)/t;
    }
    return s;
}

Circle_line circline(float s, float x0, float y0, float th0, float k){
    float x, y, th;
    x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2.0);
    y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2.0);
    th = mod2pi(th0 + k * s);
    Circle_line ret;
    ret.x = x;
    ret.y = y;
    ret.th = th;
    return ret;
}

Dubins_arc dubins_arc(float x0, float y0, float th0, float k, float L){
    
    Dubins_arc c;

    c.x0 = x0;
    c.y0 = y0;
    c.th0 = th0;
    c.k = k;
    c.L = L;
    
    Circle_line ci;
    ci = circline(L, x0, y0, th0, k);
    c.xf = ci.x; c.yf = ci.y; c.thf = ci.th;
    
    return c;
}

Dubins_curve dubins_curve(float x0, float y0, float th0, float s1, float s2, float s3, float k0, float k1, float k2){
    Dubins_curve d;
    
    d.a1 = dubins_arc(x0, y0, th0, k0, s1);
    
    d.a2 = dubins_arc(d.a1.xf, d.a1.yf, d.a1.thf, k1, s2);
    d.a3 = dubins_arc(d.a2.xf, d.a2.yf, d.a2.thf, k2, s3);
    d.L = d.a1.L + d.a2.L + d.a3.L;
    return d;
}

Scale_to_standard scale_to_standard(float x0, float y0, float th0, float xf, float yf, float thf, float Kmax){
    float dx, dy, phi, lambda, sc_th0, sc_thf, sc_Kmax;
    dx = xf - x0;
    dy = yf - y0;
    phi = atan2(dy,dx);
    lambda = hypot(dx, dy) / 2;

    sc_th0 = mod2pi(th0 - phi);
    sc_thf = mod2pi(thf - phi);
    sc_Kmax = Kmax * lambda;
    
    Scale_to_standard re;
    re.sc_th0 = sc_th0;
    re.sc_thf = sc_thf;
    re.sc_Kmax = sc_Kmax;
    re.lambda = lambda;
    return re;
}



Scale_from_standard scale_from_standard(float lambda, float sc_s1, float sc_s2, float sc_s3){
    float s1, s2, s3;
    s1 = sc_s1 * lambda;
    s2 = sc_s2 * lambda;
    s3 = sc_s3 * lambda;

    Scale_from_standard ret;
    ret.s1 = s1;
    ret.s2 = s2;
    ret.s3 = s3;

    return ret;
}

float mod2pi(float ang){
    float out = ang;
    while(out < 0){
        out = out + 2 * M_PI;
    }
    while(out >= 2 * M_PI){
        out = out - 2 * M_PI;
    }
    return out;
}

Curves_struct LSL(float sc_th0, float sc_thf, float sc_Kmax){

    float invK, C, S, temp1, sc_s1, sc_s2, sc_s3, temp2;
    bool ok;

    invK = 1/sc_Kmax;
    C = cos(sc_thf) - cos(sc_th0);
    S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    temp1 = atan2(C, S);
    sc_s1 = invK * mod2pi(temp1 - sc_th0);
    temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if(temp2 < 0){
        sc_s1 = 0; 
        sc_s2 = 0;
        sc_s3 = 0;
        ok = false;
    }else{
        sc_s2 = invK * sqrt(temp2);
        sc_s3 = invK * mod2pi(sc_thf - temp1);
        ok = true;
    }
    Curves_struct ret;
    ret.ok = ok; ret.sc_s1 = sc_s1; ret.sc_s2 = sc_s2; ret.sc_s3 = sc_s3;

    return ret;
}

Curves_struct RSR(float sc_th0, float sc_thf, float sc_Kmax){
    float invK, C, S, temp1, sc_s1, sc_s2, sc_s3, temp2;
    bool ok;

    invK = 1 / sc_Kmax;
    C = cos(sc_th0) - cos(sc_thf);
    S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    temp1 = atan2(C, S);
    sc_s1 = invK * mod2pi(sc_th0 - temp1);
    temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) -4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
    if(temp2 < 0){
        ok = false;
        sc_s1 = 0;
        sc_s2 = 0; 
        sc_s3 = 0;
    }else{
        sc_s2 = invK * sqrt(temp2);
        sc_s3 = invK * mod2pi(temp1 - sc_thf);
        ok = true;
    }
    Curves_struct ret;
    ret.ok = ok; ret.sc_s1 = sc_s1; ret.sc_s2 = sc_s2; ret.sc_s3 = sc_s3;

    return ret;
}


Curves_struct LSR(float sc_th0, float sc_thf, float sc_Kmax){
    float invK, C, S, temp1, sc_s1, sc_s2, sc_s3, temp2, temp3;
    bool ok;

    invK = 1 / sc_Kmax;
    C = cos(sc_th0) + cos(sc_thf);
    S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
    temp1 = atan2(-C, S);
    temp3 = 4 * pow(sc_Kmax, 2) - 2 +2 * cos(sc_th0 - sc_thf) +4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
    if(temp3 < 0){
        ok = false;
        sc_s1 = 0;
        sc_s2 = 0; 
        sc_s3 = 0;
    }else{
        sc_s2 = invK * sqrt(temp3);
        temp2 = -atan2(-2, sc_s2 * sc_Kmax);
        sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
        sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
        ok = true;
    }
    Curves_struct ret;
    ret.ok = ok; ret.sc_s1 = sc_s1; ret.sc_s2 = sc_s2; ret.sc_s3 = sc_s3;

    return ret;
}

Curves_struct RSL(float sc_th0, float sc_thf, float sc_Kmax){
    float invK, C, S, temp1, sc_s1, sc_s2, sc_s3, temp2, temp3;
    bool ok;

    invK = 1 / sc_Kmax;
    C = cos(sc_th0) + cos(sc_thf);
    S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
    temp1 = atan2(C, S);
    temp3 = 4 * pow(sc_Kmax,2) -2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));

    if(temp3 < 0){
        ok = false;
        sc_s1 = 0;
        sc_s2 = 0; 
        sc_s3 = 0;
    }else{
        sc_s2 = invK * sqrt(temp3);
        temp2 = atan2(2, sc_s2 * sc_Kmax);
        sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
        sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
        ok = true;
    }
    Curves_struct ret;
    ret.ok = ok; ret.sc_s1 = sc_s1; ret.sc_s2 = sc_s2; ret.sc_s3 = sc_s3;

    return ret;
}

Curves_struct RLR(float sc_th0, float sc_thf, float sc_Kmax){
    float invK, C, S, temp1, sc_s1, sc_s2, sc_s3, temp2;
    bool ok;

    invK = 1 / sc_Kmax;
    C = cos(sc_th0) - cos(sc_thf);
    S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
    temp1 = atan2(C, S);
    temp2 = 0.125 * (6 - 4 * pow(sc_Kmax, 2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
    if(abs(temp2) > 1){
        ok = false;
        sc_s1 = 0;
        sc_s2 = 0; 
        sc_s3 = 0;
    }else{
        sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
        sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
        sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
        ok = true;
    }
    
    Curves_struct ret;
    ret.ok = ok; ret.sc_s1 = sc_s1; ret.sc_s2 = sc_s2; ret.sc_s3 = sc_s3;

    return ret;
}

Curves_struct LRL(float sc_th0, float sc_thf, float sc_Kmax){
    float invK, C, S, temp1, sc_s1, sc_s2, sc_s3, temp2;
    bool ok;

    invK = 1 / sc_Kmax;
    C = cos(sc_thf) - cos(sc_th0);
    S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
    temp1 = atan2(C, S);
    temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

    if(abs(temp2) > 1){
        ok = false;
        sc_s1 = 0;
        sc_s2 = 0; 
        sc_s3 = 0;
    }else{
        sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
        sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
        sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));
        ok = true;
    }

    Curves_struct ret;
    ret.ok = ok; ret.sc_s1 = sc_s1; ret.sc_s2 = sc_s2; ret.sc_s3 = sc_s3;

    return ret;
}


