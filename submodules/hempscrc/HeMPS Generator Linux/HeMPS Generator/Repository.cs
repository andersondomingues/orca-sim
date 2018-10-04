using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace HeMPS_Generator
{
    public partial class Repository : UserControl
    {
        int repository_type; /* 0 - repository for plasma tasks
                                1 - repository for mblite tasks*/


        public Repository(int i)
        {
            InitializeComponent();
            repository_type = i;

            if (repository_type == 0)
                gbxRepository.Text = "Plasma tasks repository";
            else
                gbxRepository.Text = "MBLite tasks repository";
        }

        public ListView TaskList
        {
            get { return lvTasks; }
        }

        public int Repository_Type
        {
            get { return repository_type; }
            set { repository_type = value; }
        }

        /* Enable the drop operation */
        private void lvTasks_DragEnter(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(typeof(TreeNode)) || e.Data.GetDataPresent(typeof(ListView.SelectedListViewItemCollection)))
                e.Effect = DragDropEffects.Move;
        }

        /* Starts the drag operation */
        private void lvTasks_ItemDrag(object sender, ItemDragEventArgs e)
        {
            DoDragDrop(lvTasks.SelectedItems, DragDropEffects.Move);
        }

        /* Removes all tasks */
        public void RemoveAllTasks()
        {

			TreeNode tnApplication;
			int num_tasks = 0;
			int[] tasks = new int[lvTasks.Items.Count];
			Task[] Tasks = new Task[lvTasks.Items.Count];
				
			foreach (Task task in lvTasks.Items){
		           //if (object.ReferenceEquals(tnApp, task.Application)){  /* task.Application: refers to parent application */
		            tasks[num_tasks] = task.Index;
					Tasks[num_tasks] = task;
					num_tasks++;
			}
				
			for (; num_tasks > 0; num_tasks--){
				tnApplication = Tasks[num_tasks - 1].Application;
				tnApplication.Nodes.Add(Tasks[num_tasks-1].Text);
				tnApplication.ExpandAll();				
				lvTasks.Items.RemoveAt(tasks[num_tasks-1]);				
				
			}
			
			/*TreeNode tnApplication;
			
			int num_tasks = lvTasks.Items.Count;
			
			for (num_tasks = num_tasks; num_tasks > 0; num_tasks--)

	            // Removes all tasks from the processor and add back to the application
	            foreach (Task task in lvTasks.Items)
	            {
	                tnApplication = task.Application;	    // Retrieves the parent application
	                tnApplication.Nodes.Add(task.Text);     // Adds the task in the parent application
	                tnApplication.ExpandAll();
	                lvTasks.Items.Remove(task);             // Removes the task from the processor
	            }*/
        }

        /* Removes the draged item from the source control (aplication tree or processor) and
        add it to the current processor */
        private void lvTasks_DragDrop(object sender, DragEventArgs e)
        {

            Task task;
            int location;

            location = -1; /* Dynamic Allocation */

            /* Item draged from the application tree */
            if (e.Data.GetDataPresent(typeof(TreeNode)))
            {

                TreeNode tnTask = (TreeNode)e.Data.GetData(typeof(TreeNode));
                task = new Task(tnTask.Text, location);

                /* Sets the core type of the task based on the type of the repository */
                task.CoreType = repository_type;
                
                /* Saves the tree node parent (application tree node) */
                task.Application = tnTask.Parent;
                task.ToolTipText = tnTask.Parent.Text;

                /* Removes the task from the application tree view */
                task.Application.Nodes.Remove(tnTask);

                /* Adds the task to the current repository */
                lvTasks.Items.Add(task);

                Generator form = Form.ActiveForm as Generator;

                if (form != null)
                {
                    form.SetToOneAppStartTime(task.Application.Text);
                }
            }


        /* Itens draged from a processor */
            else if (e.Data.GetDataPresent(typeof(ListView.SelectedListViewItemCollection)))
            {
                foreach (Task t in (ListView.SelectedListViewItemCollection)e.Data.GetData(typeof(ListView.SelectedListViewItemCollection))){

                    /* Sets the core type in which the task will run based on the processor core */
                    t.CoreType = repository_type;
                    
                    /* Removes the task from the source processor */
                    t.ListView.Items.Remove(t);

                    /* Sets the new task location*/
                    t.Processor = -1;

                    /* Adds the task to the current processor */
                    lvTasks.Items.Add(t);

                    Generator form = Form.ActiveForm as Generator;

                    if (form != null)
                    {
                        form.SetToOneAppStartTime(t.Application.Text);
                    }
                }
            }
        }

        /* Remove the selected tasks */
        private void Delete_KeyDown(object sender, KeyEventArgs e)
        {

            TreeNode tnApplication;

            if (e.KeyCode == Keys.Delete)
                foreach (Task task in lvTasks.SelectedItems)
                {
                    /* Retrieves the parent application */
                    tnApplication = task.Application;

                    /* Adds the task back to the application tree */
                    tnApplication.Nodes.Add(task.Text);
                    tnApplication.ExpandAll();

                    /* Removes the task from the repository */
                    lvTasks.Items.Remove(task);
                }
        }

        /* Removes the tasks of a given application */
        public void RemoveApplicationTasks(TreeNode tnApp)
        {
			//try
			//{
				int num_tasks = 0;
				int[] tasks = new int[lvTasks.Items.Count];
				Task Task;
				
				foreach (Task task in lvTasks.Items)
		            if (object.ReferenceEquals(tnApp, task.Application)){  /* task.Application: refers to parent application */
		                	tasks[num_tasks] = task.Index;
							num_tasks++;
				}
				
				for (; num_tasks > 0; num_tasks--){
					lvTasks.Items.RemoveAt(tasks[num_tasks-1]);
				}
			//}
			/*catch(Exception ex)
			{
				MessageBox.Show(ex.ToString());
			}*/
        }


        public void SelectsTasks(TreeNode tnApp)
        {
            foreach (Task task in lvTasks.Items)
            {
                System.Drawing.Color a = task.BackColor;

                if (object.ReferenceEquals(tnApp, task.Application)) /* task.Application: refers to parent application */
                {
                    task.BackColor = System.Drawing.SystemColors.Highlight;
                    task.ForeColor = System.Drawing.SystemColors.HighlightText;
                    //task.Selected = true;
                }
                else
                {
                    task.BackColor = System.Drawing.Color.FromName("Window");
                    task.ForeColor = System.Drawing.Color.FromName("WindowText");
                }
            }
        }

          



    }
}
