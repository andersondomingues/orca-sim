/* Mapping heuristic developed by Marcelo Mandelli */

tsk taskList[500];

/*--------------------------------------------------------------------
* Premapping
*
* DESCRIPTION:
*    Allows the premapping of tasks
*
*--------------------------------------------------------------------*/

void premapping(int pe, int task)
{
	int o, k, task_type = 0;
	dependence v, dep;
	
	//for(int o = 0; o<total_tasks; o++) {puts("Task "); puts(itoa(o)); puts(" : "); puts(itoa(task_pes[o])); puts("\n");}
	for(k=0; k<taskList[task].dependences; k++) 
	{
		v = taskList[task].taskDep[k];
		//puts("\tPE tem "); puts(itoa(pe_free_pages[pe >> 4][pe & 0xF])); puts(" paginas livres!\n"); 
		if(pe_free_pages[pe >> 4][pe & 0xF]>0)
		{
			//puts("\tAnalisa a tarefa "); puts(itoa((*v).task)); puts(" que esta mapeada no PE "); puts(task_pes[(*v).task]); puts(" e é do tipo "); puts(itoa(taskList[(*v).task].type)); puts("\n");
			dep = taskList[v.task].taskDep[0];
			task_type = taskList[v.task].type;
			if(task_pes[v.task] == -1 && pe_type[pe >> 4][pe & 0xF] == task_type)
			{
				//puts("\tEntrou! "); puts(itoa((*v).task)); puts(" : "); puts(itoa(task_pes[(*v).task])); puts("\n"); 
				if(task==dep.task)
				{
					task_pes[v.task] = pe; 
					//puts("\tPremapeou a tarefa "); puts(itoa((*v).task)); puts("("); puts(itoa(task_type)); puts(") junto com a tarefa "); puts(itoa(task)); puts(" no pe "); puts(itoa(pe)); puts(" que é do tipo "); puts(itoa(pe_type[pe >> 4][pe & 0xF])); puts("\n");
					PageUsed(pe);
				}
			}
		}
		else break;
	}
}

/*--------------------------------------------------------------------
* VS
*
* DESCRIPTION:
*    Searches for the best processor with VS.
*
*--------------------------------------------------------------------*/
int SearchVS(int taskID, int type) {

	//puts("# LEC-DN - Tarefa: "); puts(itoa(taskID)); puts("("); puts(itoa(type)); puts(")\n");

   int i, x, y, xo, yo, pe, t, mhop, j;
   int xi, xf, yi, yf, left, right, top, bottom, fim, bb;
   unsigned long int min_hops, hops;
   dependence *v;
   dependence tasks[taskList[taskID].dependences];
   pe = -1;
   min_hops = 0xFFFFFFFFUL;
   
	v = taskList[taskID].taskDep;
	t = 0;
	for(i=0; i<taskList[taskID].dependences; i++) 
	{
		tasks[i].task = 0;
		if(task_pes[(*v).task] != -1) 
		{
				tasks[t].task = task_pes[(*v).task];
				tasks[t].flits = (*v).flits;
				t++;
		}
		v++;
	}
	if(t==1)
	{
		
		xo = tasks[0].task >> 4;
		yo = tasks[0].task & 0xF;
		hops = 0;
		
		xi = xo-1;
		xf = xo+1;
		yi = yo-1;
		yf = yo+1;

		if(xi<0) xi=0;
		if(yi<0) yi=0;
		if(xf>(XDIMENSION-1)) xf=XDIMENSION-1;
		if(yf>(YDIMENSION-1)) yf=YDIMENSION-1;
		
		do
		{
			for(x=xi;x<=xf;x++)
			{
				for(y=yi;y<=yf;y++)
				{
					if(pe_free_pages[x][y]>0 && pe_type[x][y]==type)
					{
						if((abs(xo-x)+abs(yo-y))==hops) 
						{
								pe=x*16+y;
						}
					}
				}
			}
				
			if(pe==-1)
			{
				hops++;
				if(xi>0) xi--;
				if(yi>0) yi--;
				if(xf<(XDIMENSION-1)) xf++;
				if(yf<(YDIMENSION-1)) yf++;
			}
			
		} while(hops<=(XDIMENSION+YDIMENSION-2) && (pe==-1));
	
    }
	else
	{
		xi = tasks[0].task >> 4;
		xf = tasks[0].task >> 4;
		yi = tasks[0].task & 0xF;
		yf = tasks[0].task & 0xF;
		
		for(i=1;i<t;i++)
		{
			if((tasks[i].task >> 4)<xi) xi = tasks[i].task >> 4;
			if((tasks[i].task >> 4)>xf) xf = tasks[i].task >> 4;
			if((tasks[i].task & 0xF)<yi) yi = tasks[i].task & 0xF;
			if((tasks[i].task & 0xF)>yf) yf = tasks[i].task & 0xF;
		}
		
		fim = 0;
		bb = 0;
		left = -1;
		right = -1;
		top = -1;
		bottom = -1;
		do
		{
			for(x=xi;x<=xf;x++)
				for(y=yi;y<=yf;y++) 
				{
					if(bb==0 || (bb==1 && ((x==left && y!=-1) || (x==right && y!=-1) || (x!=-1 && y==bottom) || (x!=-1 && y==top))))
					{
						if(pe_free_pages[x][y]>0) 
						{
							hops = 0;
							for(i=0;i<t;i++)
							{
								mhop = 0;
								xo = tasks[i].task >> 4;
								yo = tasks[i].task & 0xF;
								mhop = abs(xo-x) + abs(yo-y);
								hops = hops + mhop*tasks[i].flits;
							}
							if(hops<min_hops && pe_type[x][y]==type)
							{
								min_hops = hops;
								pe=x*16+y;
							}
						}
					}
				}
			if(pe==-1)
			{
				fim = 0;
				bb = 1;
				if(xi>0)
				{
					xi--;
					left = xi;
				}
				else
				{
					fim++;
					left = -1;
				}
				if(yi>0)
				{
					yi--;
					bottom = yi;
				}
				else
				{
					fim++;
					bottom = -1;
				}
				if(xf<(XDIMENSION-1))
				{
					xf++;
					right = xf;
				}
				else
				{ 
					fim++;
					right = -1;
				}
				if(yf<(YDIMENSION-1))
				{
					yf++;
					top = yf;
				}
				else
				{
					fim++;
					top = -1;
				}
			}
		
		} while(fim!=4 && pe==-1);
	}
	
	if(pe == -1) return EMPTY;	
	else return pe;
}

