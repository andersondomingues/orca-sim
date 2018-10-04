using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace HeMPS_Generator
{

    public partial class ProcessingElement : UserControl
    {

        int maximumTasks;	/* Maximum tasks supported */
        int address;        /* Processing elemet address */
        int x, y;           /* Processing elemet coordinates */

        public ProcessingElement()
        {
            InitializeComponent();
        }

        public int Address
        {
            get { return address; }
            set
            {
                address = value;

                if (x == 0)
                { /* first column */

                    if (rbtnMaster.Checked)
                        gbxProcessor.Text = "Processor " + 0 + "x" + y + " (Master)";
                    else
                        gbxProcessor.Text = "Processor " + 0 + "x" + y;
                }
                else
                { /* not the first column */
                    if (rbtnMaster.Checked)
                        gbxProcessor.Text = "Processor " + x + "x" + y + " (Master)";
                    else
                        gbxProcessor.Text = "Processor " + x + "x" + y;
                }
            }
        }

        public int X
        {
            get { return x; }
            set { x = value; }
        }

        public int Y
        {
            get { return y; }
            set { y = value; }
        }

        public int MaximumTasks
        {
            get { return maximumTasks; }
            set { maximumTasks = value; }
        }

        public bool Master
        {
            get { return rbtnMaster.Checked; }
            set { rbtnMaster.Checked = value; }
        }

        public bool Slave
        {
            get { return rbtnSlave.Checked; }
            set { rbtnSlave.Checked = value; }
        }

        public bool Plasma
        {
            get { return rbtnPlasma.Checked; }
            set { rbtnPlasma.Checked = value; }
        }

        public bool MBLite
        {
            get { return rbtnMBLite.Checked; }
            set { rbtnMBLite.Checked = value; }
        }

        public ListView TaskList
        {
            get { return lvTasks; }
        }

        /* Removes tasks exceeding the maximum task/slave parameter */
        public void RemoveExtraTasks()
        {

            Task task; 
            TreeNode tnApplication;

            /* Removes tasks from the processor and add it back to the application tree */
            while (lvTasks.Items.Count > maximumTasks && Slave)
            {
                task = (Task)lvTasks.Items[lvTasks.Items.Count - 1]; /* Selects the last task in the processor */
                tnApplication = task.Application;                   /* Retrieves the parent application */
                tnApplication.Nodes.Add(task.Text);                 /* Adds the task in the parent application */
                tnApplication.ExpandAll();
                lvTasks.Items.Remove(task);                         /* Removes the task from the processor */
            }
        }

        /* Removes the tasks of a given application */
        public void RemoveApplicationTasks(TreeNode tnApp)
        {
			
			int num_tasks = lvTasks.Items.Count;
			
			for (num_tasks = num_tasks; num_tasks > 0; num_tasks--)		
            	foreach (Task task in lvTasks.Items)
                	if (object.ReferenceEquals(tnApp, task.Application))  /* task.Application: refers to parent application */
                    	lvTasks.Items.Remove(task);
        }

        /* Selects as bold the tasks from the same application */
        public void SelectsTasks(TreeNode tnApp)
        {
            foreach (Task task in lvTasks.Items)
            {
                System.Drawing.Color a = task.BackColor;

                if (object.ReferenceEquals(tnApp, task.Application)) /* task.Application: refers to parent application */{
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

        /* Removes all tasks */
        public void RemoveAllTasks()
        {

            TreeNode tnApplication;

			int num_tasks = lvTasks.Items.Count;
			
			for (num_tasks = num_tasks; num_tasks > 0; num_tasks--)
			
	            /* Removes all tasks from the processor and add back to the application */
	            foreach (Task task in lvTasks.Items)
	            {
	                tnApplication = task.Application;	    /* Retrieves the parent application */
	                tnApplication.Nodes.Add(task.Text);     /* Adds the task in the parent application */
	                tnApplication.ExpandAll();
	                lvTasks.Items.Remove(task);             /* Removes the task from the processor */
	            }
        }

        /* Event rised when the check box master is checked */
        public event EventHandler NewMaster;

        /* Enable the drop operation */
        private void lvTasks_DragEnter(object sender, DragEventArgs e)
        {

            if (e.Data.GetDataPresent(typeof(TreeNode)) || e.Data.GetDataPresent(typeof(ListView.SelectedListViewItemCollection)))
                if (lvTasks.Items.Count < maximumTasks || Master)
                    e.Effect = DragDropEffects.Move;
        }

        /* Starts the drag operation */
        private void lvTasks_ItemDrag(object sender, ItemDragEventArgs e)
        {
            DoDragDrop(lvTasks.SelectedItems, DragDropEffects.Move);
        }

        /* Removes the draged item from the source control (aplication tree or processor) and
        add it to the current processor */
        private void lvTasks_DragDrop(object sender, DragEventArgs e)
        {

            Task task;
            int location;
			
			Generator form = Form.ActiveForm as Generator;

			if (form != null)
	        {
				form.projectIsSaved = false;
			}
			
            /* Drop will only be allowed in slave processors, if not, tasks cannot be dropped */
            if (rbtnSlave.Checked)
            {

                /* Drop will only be allowed in slave processores, therefore location = address */
                location = address; /* Static allocation */

                /* Item draged from the application tree */
                if (e.Data.GetDataPresent(typeof(TreeNode)))
                {

                    TreeNode tnTask = (TreeNode)e.Data.GetData(typeof(TreeNode));
                    task = new Task(tnTask.Text, location);

                    /* Sets the core type of the task based on the processor it will run */
                    if (rbtnPlasma.Checked)
                        task.CoreType = 0;
                    else
                        task.CoreType = 1;

                    /* Saves the tree node parent (application tree node) */
                    task.Application = tnTask.Parent;
                    task.ToolTipText = tnTask.Parent.Text;

                    /* Removes the task from the application tree view */
                    task.Application.Nodes.Remove(tnTask);

                    /* Adds the task to the current processor */
                    lvTasks.Items.Add(task);

                    //Generator form = Form.ActiveForm as Generator;

                    if (form != null)
                    {
                        form.SetToZeroAppStartTime(task.Application.Text);
                    }
                }


            /* Itens draged from another processor */
                else if (e.Data.GetDataPresent(typeof(ListView.SelectedListViewItemCollection)))
                {
                    foreach (Task t in (ListView.SelectedListViewItemCollection)e.Data.GetData(typeof(ListView.SelectedListViewItemCollection)))

                        if (lvTasks.Items.Count < maximumTasks)
                        {

                            /* Sets the core type in which the task will run based on the processor core */
                            if (rbtnPlasma.Checked)
                                t.CoreType = 0;
                            else
                                t.CoreType = 1;

                            /* Removes the task from the source processor */
                            t.ListView.Items.Remove(t);

                            /* Sets the new task location*/
                            t.Processor = location;

                            /* Adds the task to the current processor */
                            lvTasks.Items.Add(t);

                            //Generator form = Form.ActiveForm as Generator;

                            if (form != null)
                            {
                                form.SetToZeroAppStartTime(t.Application.Text);
                            }
                        }
                       
                    //else
                    //break;
                }
            }
            else
                /* Warns the user that tasks cannot be dropped in the Master Processor */
                MessageBox.Show("Tasks cannot be dropped in the Master processor. To add tasks to the repository, drag them to the repository panel.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
        }

        /* Changes the task list back color */
        private void cbxEnable_CheckedChanged(object sender, EventArgs e)
        {

           lvTasks.BackColor = System.Drawing.SystemColors.Window;
           
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

                    /* Removes the task from the processor */
                    lvTasks.Items.Remove(task);
                }
        }

        /* Remove the selected tasks */
        private void btnRemove_Click(object sender, EventArgs e)
        {

            TreeNode tnApplication;

            foreach (Task task in lvTasks.SelectedItems)
            {
                /* Retrieves the parent application */
                tnApplication = task.Application;

                /* Adds the task back to the application tree */
                tnApplication.Nodes.Add(task.Text);
                tnApplication.ExpandAll();

                /* Removes the task from the processor */
                lvTasks.Items.Remove(task);
            }
        }

        /* Warns the edidor that a new master was selected */
        private void rbtnMaster_CheckedChanged(object sender, EventArgs e)
        {

            /* Sets the new master processor */
            if (rbtnMaster.Checked)
            {
                rbtnPlasma.Checked = true;
                rbtnMBLite.Enabled = false;
                rbtnPlasma.Enabled = false;

                lvTasks.BackColor = System.Drawing.SystemColors.GradientActiveCaption;
                
                
                if (x == 0) /* first column */
                    this.gbxProcessor.Text = "Processor " + 0 + "x" + y + " (Master)";
                  
                else /* not the first column */
                    this.gbxProcessor.Text = "Processor " + x + "x" + y + " (Master)";
                    
                /* Adds the '(Master)' indication in the label */

                NewMaster(this, EventArgs.Empty);

                /* Remove all tasks from the new master processor */
                RemoveAllTasks();

            }
            /* Removes extra tasks when the processor changes from master to slave */
            else
            {
                rbtnMBLite.Enabled = true;
                rbtnPlasma.Enabled = true;

                RemoveExtraTasks();
                lvTasks.BackColor = System.Drawing.SystemColors.Window;
                
                /* Removes the the '(Master)' from the label */
                if (x == 0) /* first column */
                    this.gbxProcessor.Text = "Processor " + x + "x" + y;

                else /* not the first column */
                    this.gbxProcessor.Text = "Processor " + x + "x" + y;
            }
        }

        private void gbxProcessor_Enter(object sender, EventArgs e)
        {

        }

        private void rbtnPlasma_CheckedChanged(object sender, EventArgs e)
        {
            foreach (Task task in lvTasks.Items)
            {
                if (rbtnPlasma.Checked)
                    task.CoreType = 0;
                else
                    task.CoreType = 1;
            }
        }

    }
}