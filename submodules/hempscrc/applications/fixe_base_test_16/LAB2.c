#include <task.h>
#include <stdlib.h>


#define FIXE 4 /*nb de chiffres après la virgule*/
#define MAX 1000000000 /*10^PU*/
#define PU 9 /*puissance de 10 max supportée ici 2³¹->2 000 000 000 donc 10�?/
#define size 16
#define data 64
#define data_val 640000 /*valeur de data et size avec FIXE chiffres après la virgule*/
#define size_val 160000
#define region 1



int mult(int a, int b)
{
	int i,res;
	int cpt1=0;  /*détermine l'ordre de grandeur en puissance de 10 de a*/
	int cpt2=0;  /*détermine l'ordre de grandeur en puissance de 10 de b*/
	int cpt =2*FIXE; /*compte le nombre de décimales*/
	int a2,b2;

	a2=a;
	b2=b;

	/*détermine l'ordre de grandeur en puissance de 10 de a*/
	while(a2!=0)
	{
		a2=a2/10;
		cpt1++;
	}
	cpt1--;

	/*détermine l'ordre de grandeur en puissance de 10 de b*/
	while(b2!=0)
	{
		b2=b2/10;
		cpt2++;
	}
	cpt2--;

	/*divise a et b en conséquence lorsque ceux-ci sont trop grand*/
	/*on sacrifie en précision pour pouvoir effectuer le calcul*/
	while((cpt1+cpt2)>=PU-1)
	{
		a=a/10;
		cpt1--;
		cpt--;
		if((cpt1+cpt2)>=PU-1)
		{
			b=b/10;
			cpt2--;
			cpt--;
		}
	}

	/*calcul de la multiplication*/
	res=a*b;

	/*retire les derniers chiffres après la virgule pour n'en garder que FIXE*/
	if(cpt>FIXE)
	{
		for(i=0;i<(cpt-FIXE);i++)
		{
			res=res/10;
		}
	}
	else
	{
		while(cpt<FIXE)
		{
			res=res*10;
			cpt++;
		}
	}
	return res;
}

int div(int a, int b)
{
	int i;
	int res=0;
	int entier,nb=1;
	int reste;
	int cpt=0;
	int cpt2=FIXE;
	int cpt3=0;
	int reste2;

	if (b==0)
	{
		return -1;
	}

    /*récupère la partie entière du résultat de la division*/
	entier=a/b;

	/*calcul le résultat entier avec FIXE zéros derrières*/
	for(i=0;i<FIXE;i++)
		{
			entier=entier*10;
		}

	if(a!=b)
	{
		reste=sub(a,mult(entier,b));
	}
	else
	{return entier;}

	if(reste==0)
	{return entier;}

	for(i=0;i<FIXE;i++)
	{
		nb=nb*10;
	}
	reste2=reste;
	while(reste2!=0)
	{
		reste2=reste2/10;
		cpt++;
	}
	cpt--;
		/*calcul nb*a jusqu'à ce qu'on ait un res>(1000*entier) on a alors les quatre décimales*/
		/*while((res<val)&&((reste*nb)<MAX))
		{
			nb=nb*10;
			res=reste*nb/b;
		} */
	while((cpt+cpt2)>=PU)
	{
		nb=nb/10;
		cpt2--;
		cpt3++;
		if((cpt+cpt2)>=PU)
		{
			reste=reste/10;
			cpt--;
			cpt3++;
		}
	}
	res=nb*reste/b;
	for(i=0;i<cpt3;i++)
	{
		res=res*10;
	}
	/*res=normalise(res);*/
	res=add(res,entier);
	return res;
}

int pow(int x,int y)
{

	int puissance = 10000;


	int i;

	for(i = 0 ; i < (y/10000); i++)
		puissance = mult(puissance,x);
	return puissance;

}


int	racine(int val,int expo)
{
	int				i,j;
    int ex=expo;
	int				result=10000;
	int				value;

	if (ex<0) ex=-ex;
	for (i=0;i<10;i++)
	{
		value=10000;
		for (j=0;j<(ex/10000);j++)
            value=mult(value,result);
		result=div(add(mult(value,sub(ex,10000)),val),mult(ex,div(value,result)));
	}
	if (expo<0)
       result=div(10000,result);
	return result;
}



Message msg1;

void lab(int* sum,int* LAB)
{
    int val_a,val_l,val_b,var_X,var_Y,var_Z;

        /*calcul de L*/
	    var_Y = div(sum[1],1000000);
	    if (var_Y>89)   /*valeur th�orique : 0.008856*/
        {

                   var_Y = racine(var_Y,30000);

        }
	    else
		           var_Y = add(mult( 77870,var_Y ),div(160000,1160000));

	    val_l=sub(mult( 1160000,var_Y ),160000);
	    LAB[0]=val_l;

     /*calcul de a*/
	    var_X=div(sum[0],950470);  /*vrai valeur 95.047*/
	    var_Y=div(sum[1],1000000);
	     if (var_X > 89)  /*valeur th�orique : 0.008856*/
		           var_X = racine(var_X,30000);
	     else
		           var_X = div(add(mult(9033000,var_X),160000),1160000);
	     if (var_Y > 89)  /*valeur th�orique : 0.008856*/
	                var_Y = racine(var_Y,30000);
	      else
	                var_Y = div(add(mult(9033000,var_Y),160000),1160000);

	    val_a=mult(5000000,sub(var_X,var_Y));
	    LAB[1]=val_a;

        /*calcul de b*/
	      var_Y=div(sum[1],1000000);
	      var_Z=div(sum[2],1088830); /*vrai valeur 108.883*/
	      if (var_Y > 89) /*valeur th�orique : 0.008856*/
	               var_Y = racine(var_Y,30000);
	      else
	               var_Y = div(add(mult(9033000,var_Y),160000),1160000);
	      if (var_Z>89) /*valeur th�orique : 0.008856*/
                   var_Z= racine(var_Z,30000);
	      else
     		       var_Z = div(add(mult(9033000,var_Z),160000),1160000);
          val_b=mult(2000000,sub(var_Y,var_Z));
          LAB[2]=val_b;

}



int main()
{
	Echo("start LAB2");
	Echo(itoa(GetTick()));


	int i;
	int LAB[3]={0,0,0};

	Receive(&msg1,XYZ2);

    lab(msg1.msg,LAB);

    for (i=0;i<3;i++)
    {
        msg1.msg[i]=LAB[i];
        Echo(fixetoa(LAB[i]));
    }

    Send(&msg1,DLAB);

    Echo(itoa(GetTick()));
    Echo("Communication LAB2 finished.");

exit();
}
