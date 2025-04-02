import gurobi.*;
import java.io.IOException;
public class Main {
    public static void main(String[] args) throws IOException{
        try {

            Parameter instance = new Parameter();
            instance.initParams("D:\\学习资料\\vrp求解范例\\solomon-100\\In\\c201.txt");
            instance.dist[0][instance.nbClients+1] = instance.veribig;
            for (int i = 0; i < instance.speed.length; i++) {
                instance.travelTime[i][0][instance.nbClients+1] = instance.veribig;
            }
            double maxRechargeTime = instance.battery*(0.2/44+0.6/22+0.2/11);
            double[] a = new double[4];
            a[0] = 0;
            a[1] = 0.2*instance.battery;
            a[2] = 0.8*instance.battery;
            a[3] = instance.battery;
            double[] c = new double[4];
            c[0] = 0;
            c[1] = 0.2*instance.battery/11;
            c[2] = 0.6*instance.battery/22+c[1];
            c[3] = maxRechargeTime;
            for (int i = 0; i < instance.nbClients+2; i++) {
                instance.demand[i] = instance.demand[i]*40;
            }
            for (int i = 0; i < instance.rechargeStations.length; i++) {
                instance.demand[instance.rechargeStations[i]] = 0;
            }
        /*for (int i = 0; i < instance.nbClients+2; i++) {
            for (int j = 0; j < instance.nbClients+2; j++) {
                for (int k = 0; k < instance.speed.length; k++) {
                    if(instance.travelTime[k][i][j]<100){
                        System.out.println("h:"+k+",i:"+i+",j:"+j+",time:"+instance.travelTime[k][i][j]);
                        System.out.println("i:"+i+",j:"+j+",dist:"+ instance.dist[i][j]);
                    }
                }
            }
        }*/
            instance.capacity =3650;
            double[] pi = new double[instance.nbClients+2];
            for (int i = 0; i < instance.nbClients+2; i++) {
                pi[i] = 1000;
            }
            for (int i = 0; i < instance.rechargeStations.length; i++) {
                pi[instance.rechargeStations[i]] = 0;
            }
            pi[0] = 0;
            pi[instance.nbClients+1] = 0;
            instance.demand[instance.nbClients+1] = 0;
            GRBEnv env = new GRBEnv();
            GRBModel model = new GRBModel(env);
            model.set(GRB.DoubleParam.TimeLimit,7200);
            GRBVar[] Delta = new GRBVar[instance.rechargeStations.length];
            GRBVar[] y = new GRBVar[instance.nbClients+2];
            GRBVar[] Y = new GRBVar[instance.nbClients+2];
            GRBVar[] s = new GRBVar[instance.rechargeStations.length];
            GRBVar[] o = new GRBVar[instance.rechargeStations.length];
            GRBVar[] p = new GRBVar[instance.rechargeStations.length];
            GRBVar[][] we = new GRBVar[instance.nbClients+2][instance.nbClients+2];
            GRBVar[][] alpha = new GRBVar[instance.rechargeStations.length][4];
            GRBVar[][] z = new GRBVar[instance.rechargeStations.length][4];
            GRBVar[][] lambd = new GRBVar[instance.rechargeStations.length][4];
            GRBVar[][] w = new GRBVar[instance.rechargeStations.length][4];
            GRBVar[][] x = new GRBVar[instance.nbClients+2][instance.nbClients+2];
            GRBVar[][] e = new GRBVar[instance.nbClients+2][instance.nbClients+2];
            GRBVar[][][] delta = new GRBVar[instance.speed.length][instance.nbClients+2][instance.nbClients+2];
            GRBVar[] u = new GRBVar[instance.nbClients+2];
            //delta, x, we ,e
            for (int i = 0; i < instance.nbClients+2; i++) {
                y[i] = model.addVar(0,instance.battery,0,GRB.CONTINUOUS,"y_"+i);
                Y[i] = model.addVar(0,instance.battery,0,GRB.CONTINUOUS,"Y_"+i);
                u[i] = model.addVar(0,instance.capacity,0,GRB.CONTINUOUS,"u_"+i);
                for (int j = 0; j < instance.nbClients+2; j++) {
                    x[i][j] = model.addVar(0,1,0,GRB.BINARY,"x_"+i+"_"+j);
                    we[i][j] = model.addVar(0,instance.capacity,0,GRB.CONTINUOUS,"we_"+i+"_"+j);
                    e[i][j] = model.addVar(0,instance.battery,0,GRB.CONTINUOUS,"e_"+i+"_"+j);
                    for (int h = 0; h < instance.speed.length; h++) {
                        delta[h][i][j] = model.addVar(0,1,0,GRB.BINARY,"delta_"+h+"_"+i+"_"+j);
                    }
                }
            }

            //Delta, o, s, p, alpha, z, lambd, w
            for (int i = 0; i < instance.rechargeStations.length; i++) {
                Delta[i] = model.addVar(0,maxRechargeTime,0,GRB.CONTINUOUS,"Delta_"+i);
                o[i] = model.addVar(0,instance.battery,0,GRB.CONTINUOUS,"o_"+i);
                s[i] = model.addVar(0,maxRechargeTime,0,GRB.CONTINUOUS,"s_"+i);
                p[i] = model.addVar(0,maxRechargeTime,0,GRB.CONTINUOUS,"p_"+i);
                for (int j = 0; j < 4; j++) {
                    alpha[i][j] = model.addVar(0,1,0,GRB.CONTINUOUS,"alpha_"+i+"_"+j);
                    z[i][j] = model.addVar(0,1,0,GRB.BINARY,"z_"+i+"_"+j);
                    lambd[i][j] = model.addVar(0,1,0,GRB.CONTINUOUS,"lambd_"+i+"_"+j);
                    w[i][j] = model.addVar(0,1,0,GRB.BINARY,"w_"+i+"_"+j);
                }
            }


            GRBConstr[] c3b = new GRBConstr[instance.nbClients+2];
            GRBConstr[] c3c = new GRBConstr[instance.nbClients+2];
            GRBConstr[] c3d = new GRBConstr[instance.rechargeStations.length];
            GRBConstr c3e;
            GRBConstr[][] c3o = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
            //GRBConstr c3f;
            GRBConstr[] c3g = new GRBConstr[instance.nbClients+2];
            GRBConstr[][] c3h = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
            GRBConstr[][] c3i = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
            GRBConstr[][] c3j1 = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
            GRBConstr[][] c3j2 = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
            GRBConstr[][] c3k1 = new GRBConstr[instance.nbClients+2][instance.rechargeStations.length];
            GRBConstr[][] c3k2 = new GRBConstr[instance.nbClients+2][instance.rechargeStations.length];
            GRBConstr[] c3l = new GRBConstr[instance.nbClients+2];
            GRBConstr c3m;
            GRBConstr[] c3n1 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n2 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n3 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n4 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n5 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n6 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n7 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[][] c3n8 = new GRBConstr[instance.rechargeStations.length][4];
            GRBConstr[] c3n9 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n10 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n11 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n12 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n13 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n14 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[][] c3n15 = new GRBConstr[instance.rechargeStations.length][4];
            GRBConstr[] c3n16 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[] c3n17 = new GRBConstr[instance.rechargeStations.length];
            GRBConstr[][] c6 = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
            GRBConstr[] c7 = new GRBConstr[instance.nbClients+2];
            GRBConstr[][] c8 = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
            GRBConstr[][] c9 = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
            GRBConstr[][] c10 = new GRBConstr[instance.nbClients+2][instance.nbClients+2];
           /* for (int i = 1; i <instance.nbClients+1 ; i++) {
                boolean isStation = false;
                for (int j = 0; j < instance.rechargeStations.length; j++) {
                    if(i==instance.rechargeStations[j]){
                        isStation = true;
                        break;
                    }
                }
                if(isStation){
                    continue;
                }else {
                    GRBLinExpr exrp1 = new GRBLinExpr();
                    for (int j = 1; j < instance.nbClients+2; j++) {
                        exrp1.addTerm(1,x[i][j]);
                    }


                    c3b[i] = model.addConstr(exrp1,GRB.LESS_EQUAL,1,"c3b_"+i);
                }

            }*/
            for (int i = 1; i <instance.nbClients+1 ; i++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                GRBLinExpr expr2 = new GRBLinExpr();
                for (int j = 1; j <instance.nbClients+1 ; j++) {
                    expr1.addTerm(1,x[j][i]);
                    expr2.addTerm(1,x[i][j]);
                }
                expr1.addTerm(1,x[0][i]);
                expr2.addTerm(1,x[i][instance.nbClients+1]);

                /*for (int j = 0; j < instance.rechargeStations.length; j++) {
                    expr1.addTerm(-1,x[instance.rechargeStations[j]][i]);
                    expr2.addTerm(-1,x[i][instance.rechargeStations[j]]);
                }*/
                c3c[i] = model.addConstr(expr1,GRB.EQUAL,expr2,"c3c_"+i);
            }
            /*for (int i = 0; i < instance.rechargeStations.length; i++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                GRBLinExpr expr2 = new GRBLinExpr();
                for (int j = 1; j <instance.nbClients+1 ; j++) {
                    expr1.addTerm(1,x[j][instance.rechargeStations[i]]);
                    expr2.addTerm(1,x[instance.rechargeStations[i]][j]);
                }
                expr1.addTerm(1,x[0][i]);
                expr2.addTerm(1,x[i][instance.nbClients+1]);
                expr1.addTerm(-1,x[i][i]);
                expr2.addTerm(-1,x[i][i]);
                for (int j = 0; j < instance.rechargeStations.length; j++) {
                    expr1.addTerm(-1,x[instance.rechargeStations[j]][i]);
                    expr2.addTerm(-1,x[i][instance.rechargeStations[j]]);
                }
                c3d[i] = model.addConstr(expr1,GRB.EQUAL,expr2,"c3d_"+i);
            }*/
            GRBLinExpr expr = new GRBLinExpr();
            for (int i = 1; i < instance.nbClients+1; i++) {
                expr.addTerm(1,x[0][i]);
            }
            c3e = model.addConstr(expr,GRB.EQUAL,1,"c3e");
            GRBLinExpr expr0 = new GRBLinExpr();
            for (int i = 0; i < instance.rechargeStations.length; i++) {
                for (int j = 1; j < instance.nbClients+2; j++) {
                    boolean isStation = false;
                    for (int k = 0; k < instance.rechargeStations.length; k++) {
                        if(j == instance.rechargeStations[k]){
                            isStation = true;
                            break;
                        }
                    }
                    if(!isStation){
                        expr0.addTerm(1,x[instance.rechargeStations[i]][j]);
                        //System.out.println("yes");
                    }
                }
            }
            GRBConstr c3f = model.addConstr(expr0,GRB.LESS_EQUAL,1,"c3f");
           /* for (int i =1; i < instance.nbClients+1; i++) {
                GRBLinExpr expr1 = new GRBLinExpr();
                GRBLinExpr expr2 = new GRBLinExpr();
                GRBLinExpr expr3 = new GRBLinExpr();
                for (int j = 1; j < instance.nbClients+2; j++) {
                    expr1.addTerm(1,we[j][i]);
                    expr1.addTerm(-1,we[i][j]);
                    expr3.addTerm(instance.demand[i],x[j][i]);
                }
                expr1.addTerm(1,we[0][i]);
                expr1.addTerm(-1,we[i][instance.nbClients+1]);
                expr1.addTerm(-1,we[i][i]);
                expr1.addTerm(1,we[i][i]);
                expr3.addTerm(instance.demand[i],x[0][i]);
                c3g[i] = model.addConstr(expr1,GRB.GREATER_EQUAL,expr3,"c3g_"+i);
            }*/
            for (int i = 0; i < instance.nbClients+2; i++) {
                for (int j = 0; j < instance.nbClients+2; j++) {
                    /*GRBLinExpr expr1 = new GRBLinExpr();

                    expr1.addTerm(1,we[i][j]);
                    expr1.addTerm(-instance.capacity,x[i][j]);
                    c3h[i][j] = model.addConstr(expr1,GRB.LESS_EQUAL,0,"c3h_"+i);*/
                    GRBLinExpr expr2 = new GRBLinExpr();
                    expr2.addTerm(1,e[i][j]);
                    expr2.addTerm(-instance.Phi[i][j], we[i][j]);
                    for (int h = 0; h < instance.speed.length; h++) {
                        expr2.addTerm(-instance.beta[i][j]*instance.speed[h]*instance.speed[h],delta[h][i][j]);
                        expr2.addTerm(-instance.gamma[i][j]/instance.speed[h],delta[h][i][j]);
                    }
                    expr2.addTerm(-instance.epsilon[i][j],x[i][j]);
                    c3i[i][j] = model.addConstr(expr2,GRB.EQUAL,0,"c3i_"+i+"_"+j);
                    GRBLinExpr expr3 = new GRBLinExpr();
                    GRBLinExpr expr4 = new GRBLinExpr();
                    //GRBLinExpr expr5 = new GRBLinExpr();
                    GRBLinExpr expr6 = new GRBLinExpr();
                    GRBLinExpr expr7 = new GRBLinExpr();
                    //GRBLinExpr expr8 = new GRBLinExpr();
                    boolean isStation = false;
                    for (int k = 0; k < instance.rechargeStations.length; k++) {
                        if(j == instance.rechargeStations[k]){
                            isStation = true;
                            expr6.addTerm(1,e[i][j]);
                            expr6.addTerm(instance.battery,x[i][j]);
                            expr6.addTerm(1,y[j]);
                            expr6.addTerm(-1,Y[i]);
                            c3k1[i][k] = model.addConstr(expr6,GRB.LESS_EQUAL,instance.battery,"c3k1_"+i+"_"+"j");
                            expr7.addTerm(1,e[i][j]);
                            expr7.addTerm(-instance.battery,x[i][j]);
                            expr7.addTerm(1,y[j]);
                            expr7.addTerm(-1,Y[i]);
                            c3k2[i][k] = model.addConstr(expr7,GRB.GREATER_EQUAL,-instance.battery,"c3k2_"+i+"_"+j);
                            break;
                        }
                    }
                    if(!isStation){
                        expr3.addTerm(1,e[i][j]);
                        expr3.addTerm(instance.battery,x[i][j]);
                        expr3.addTerm(-1,Y[i]);
                        expr3.addTerm(1,Y[j]);
                        c3j1[i][j] = model.addConstr(expr3,GRB.LESS_EQUAL,instance.battery,"c3j1_"+i+"_"+j);
                        expr4.addTerm(1,e[i][j]);
                        expr4.addTerm(-instance.battery,x[i][j]);
                        expr4.addTerm(-1,Y[i]);
                        expr4.addTerm(1,Y[j]);
                        c3j2[i][j] = model.addConstr(expr4,GRB.GREATER_EQUAL,-instance.battery,"c3j2_"+i+"_"+j);

                    }

                }
                c3l[i] = model.addConstr(Y[i],GRB.GREATER_EQUAL,e[i][instance.nbClients+1],"c3l_"+i);

            }
            c3m = model.addConstr(Y[0],GRB.EQUAL,instance.battery,"c3m");
            for (int i = 0; i < instance.rechargeStations.length; i++) {
                c3n1[i] = model.addConstr(y[instance.rechargeStations[i]],GRB.LESS_EQUAL,o[i],"c3n1_"+i);
                c3n2[i] = model.addConstr(Y[instance.rechargeStations[i]],GRB.EQUAL,o[i],"c3n2_"+i);
                GRBLinExpr expr1 = new GRBLinExpr();
                GRBLinExpr expr2 = new GRBLinExpr();
                GRBLinExpr expr3 = new GRBLinExpr();
                GRBLinExpr expr4 = new GRBLinExpr();
                GRBLinExpr expr5 = new GRBLinExpr();
                GRBLinExpr expr6 = new GRBLinExpr();
                GRBLinExpr expr7 = new GRBLinExpr();
                GRBLinExpr expr8 = new GRBLinExpr();
                GRBLinExpr expr9 = new GRBLinExpr();
                GRBLinExpr expr10 = new GRBLinExpr();
                GRBLinExpr expr11 = new GRBLinExpr();
                GRBLinExpr expr12 = new GRBLinExpr();
                for (int j = 0; j < 4; j++) {
                    expr1.addTerm(a[j],alpha[i][j]);
                    expr2.addTerm(c[j],alpha[i][j]);
                    expr3.addTerm(1,alpha[i][j]);
                    expr7.addTerm(a[j],lambd[i][j]);
                    expr8.addTerm(c[j],lambd[i][j]);
                }
                c3n3[i] = model.addConstr(y[instance.rechargeStations[i]],GRB.EQUAL,expr1,"c3n3_"+i);
                c3n4[i] = model.addConstr(s[i],GRB.EQUAL,expr2,"c3n4_"+i);
                for (int j = 1; j <4 ; j++) {
                    expr4.addTerm(1,z[i][j]);
                    expr5.addTerm(1,z[i][j]);
                    expr9.addTerm(1,lambd[i][j]);
                    expr10.addTerm(1,w[i][j]);
                    expr11.addTerm(1,w[i][j]);
                }
                c3n5[i] = model.addConstr(expr3,GRB.EQUAL,expr4,"c3n5_"+i);
                for (int j = 0; j < instance.nbClients+2; j++) {
                    expr6.addTerm(1,x[instance.rechargeStations[i]][j]);
                    expr12.addTerm(1,x[instance.rechargeStations[i]][j]);
                }
                c3n6[i] = model.addConstr(expr5,GRB.EQUAL,expr6,"c3n6_"+i);
                c3n7[i] = model.addConstr(alpha[i][0],GRB.LESS_EQUAL,z[i][1],"c3n7_"+i);

                for (int j = 1; j <3 ; j++) {
                    GRBLinExpr expr13 = new GRBLinExpr();
                    expr13.addTerm(1,z[i][j]);
                    expr13.addTerm(1,z[i][j+1]);
                    GRBLinExpr expr14 = new GRBLinExpr();
                    expr14.addTerm(1,w[i][j]);
                    expr14.addTerm(1,w[i][j+1]);
                    c3n8[i][j] = model.addConstr(alpha[i][j],GRB.LESS_EQUAL,expr13,"c3n8_"+i);
                    c3n15[i][j] = model.addConstr(lambd[i][j],GRB.LESS_EQUAL,expr14,"c3n15_"+i);
                }
                c3n9[i] = model.addConstr(alpha[i][3],GRB.LESS_EQUAL,z[i][3],"c3n9_"+i);
                c3n10[i] = model.addConstr(o[i],GRB.EQUAL,expr7,"c3n10_"+i);
                c3n11[i] = model.addConstr(p[i],GRB.EQUAL,expr8,"c3n11_"+i);
                c3n12[i] = model.addConstr(expr9,GRB.EQUAL,expr10,"c3n12_"+i);
                c3n13[i] = model.addConstr(expr11,GRB.EQUAL,expr12,"c3n13_"+i);
                c3n14[i] = model.addConstr(lambd[i][0],GRB.LESS_EQUAL,w[i][1],"c3n14_"+i);
                c3n16[i] = model.addConstr(lambd[i][3],GRB.LESS_EQUAL,w[i][3],"c3n16_"+i);
                GRBLinExpr expr13 = new GRBLinExpr();
                expr13.addTerm(1,p[i]);
                expr13.addTerm(-1,s[i]);
                c3n17[i] = model.addConstr(Delta[i],GRB.EQUAL,expr13,"c3n17_"+i);

            }
            for (int i = 0; i < instance.nbClients+2; i++) {
                GRBLinExpr expr2 = new GRBLinExpr();
                for (int j = 0; j < instance.nbClients+2; j++) {
                    GRBLinExpr expr1 = new GRBLinExpr();

                    GRBLinExpr expr3 = new GRBLinExpr();
                    GRBLinExpr expr4 = new GRBLinExpr();
                    GRBLinExpr expr5 = new GRBLinExpr();
                    expr1.addTerm(-1,u[j]);
                    expr1.addTerm(1,u[i]);
                    expr1.addTerm(-instance.demand[i],x[i][j]);
                    expr1.addTerm(-instance.capacity,x[i][j]);
                    c6[i][j] = model.addConstr(expr1,GRB.GREATER_EQUAL,-instance.capacity,"c6_"+i+"_"+j);
                    expr2.addTerm(instance.capacity,x[i][j]);
                    expr3.addTerm(instance.capacity,x[i][j]);
                    expr3.addTerm(-1,we[i][j]);
                    c8[i][j] = model.addConstr(expr3,GRB.GREATER_EQUAL,0,"c8_"+i+"_"+j);
                    expr4.addTerm(1,u[i]);
                    expr4.addTerm(-1,we[i][j]);
                    c9[i][j] = model.addConstr(expr4,GRB.GREATER_EQUAL,0,"c9_"+i+"_"+j);
                    expr5.addTerm(1,we[i][j]);
                    expr5.addTerm(-1,u[i]);
                    expr5.addTerm(-instance.capacity,x[i][j]);
                    c10[i][j] = model.addConstr(expr5,GRB.GREATER_EQUAL,-instance.capacity,"c10_"+i+"_"+j);
                }
                expr2.addTerm(-1,u[i]);
                c7[i] = model.addConstr(expr2,GRB.GREATER_EQUAL,0,"c7_"+i);
            }




            for (int i = 0; i < instance.nbClients+2; i++) {
                for (int j = 0; j < instance.nbClients+2; j++) {
                    GRBLinExpr expr1 = new GRBLinExpr();
                    for (int k = 0; k < instance.speed.length; k++) {
                        expr1.addTerm(1,delta[k][i][j]);
                    }
                    c3o[i][j] = model.addConstr(expr1,GRB.EQUAL,x[i][j],"c3o_"+i+"_"+j);
                }
            }



            GRBLinExpr expr1 = new GRBLinExpr();
            for (int i = 0; i < instance.nbClients+2; i++) {
                for (int j = 0; j < instance.nbClients+2; j++) {
                    expr1.addTerm(-pi[i],x[i][j]);
                    for (int k = 0; k < instance.speed.length; k++) {
                        expr1.addTerm(instance.travelTime[k][i][j],delta[k][i][j]);
                    }
                }
            }
            for (int i = 0; i < instance.rechargeStations.length; i++) {
                expr1.addTerm(3600,Delta[i]);
                for (int j = 1; j < instance.nbClients+2; j++) {
                    expr1.addTerm(instance.waitingTime[i],x[i][j]);
                    expr1.addTerm(instance.varTheta,x[i][j]);
                }
            }
            model.setObjective(expr1,GRB.MINIMIZE);
            model.optimize();
            for(GRBVar v:model.getVars()){
                double sol = v.get(GRB.DoubleAttr.X);
                if((sol > 0.0001)&&(sol<GRB.INFINITY)){
                    System.out.println(v.get(GRB.StringAttr.VarName)+":"+v.get(GRB.DoubleAttr.X));
                }
            }
            model.dispose();
            env.dispose();
            System.out.println(instance.dist[instance.nbClients+1][0]);
            /*for (int i = 0; i < instance.nbClients+2; i++) {
                System.out.println("demnad[:"+i+"]:"+instance.demand[i]);
            }*/
            System.out.println(instance.travelTime[1][0][10]+ instance.travelTime[0][10][8]+ instance.travelTime[0][8][9]+instance.travelTime[0][9][6]+instance.travelTime[0][6][4]+instance.travelTime[1][4][7]+instance.travelTime[0][7][5]+instance.travelTime[1][5][3]+instance.travelTime[0][3][11]);
            System.out.println(instance.travelTime[2][0][5]+ instance.travelTime[2][5][7]+ instance.travelTime[1][7][4]+instance.travelTime[1][4][3]+instance.travelTime[2][3][6]+instance.travelTime[2][6][8]+instance.travelTime[2][8][9]+instance.travelTime[0][9][10]+instance.travelTime[1][10][11]+instance.waitingTime[3]+instance.varTheta+0.0025261486869261507-35);
            System.out.println(instance.travelTime[1][0][10]+ instance.travelTime[0][10][8]+ instance.travelTime[0][8][9]+instance.travelTime[0][9][6]+instance.travelTime[0][6][4]+instance.travelTime[1][4][7]+instance.travelTime[0][7][5]+instance.travelTime[1][5][3]+instance.travelTime[0][3][11]+instance.waitingTime[3]+instance.varTheta+0.0025261486869261507-35);



        }catch (GRBException e){
            System.out.println("Error code:" +e.getErrorCode()+"."+e.getMessage());
        }

    }
}
