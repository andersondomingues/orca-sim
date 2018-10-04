using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace HeMPS_Generator
{
    public partial class TaskProperties : Form {

        private Task task;
        
        public TaskProperties(Task t) {

            task = t;
            
            InitializeComponent();

            Text = "Task " + t.Text + " properties";
            nudPriority.Value = t.Priority;
            nudTimeSlice.Value = t.TimeSlice;
        }

        /* Sets the task priorities */
        private void btnOK_Click(object sender, EventArgs e) {
            task.Priority = (int)nudPriority.Value;
            task.TimeSlice = (int)nudTimeSlice.Value;
            DialogResult = DialogResult.OK;
        }

        /* Does nothing */
        private void btnCancel_Click(object sender, EventArgs e) {
            DialogResult = DialogResult.Cancel;
        }
    }
}
