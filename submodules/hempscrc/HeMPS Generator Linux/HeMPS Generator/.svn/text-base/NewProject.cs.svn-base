using System;
using System.IO;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace HeMPS_Generator
{
    public partial class NewProject : Form
    {
        string name;
        string path;
		DialogResult result;
		
        public NewProject()
        {

            InitializeComponent();
        }

        public string NAme
        {
            get { return name; }
            set { name = value; }
        }

        public string Path
        {
            get { return path; }
            set { path = value; }
        }

		public DialogResult Result
		{
			get {return result;}
		}
		
        private void btnOK_Click(object sender, EventArgs e)
        {
            bool invalid = false;
            name = tbName.Text;
            path = tbPath.Text;
            
            if (name.Contains("\\") || name.Contains("/"))
            {
                MessageBox.Show("There are invalid characters in the project name!", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                invalid = true;
            }
            if (name == "") 
            {
                MessageBox.Show("The project name is empty!", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                invalid = true;
            }
            if (path == "")
            {
                MessageBox.Show("The project path is empty!", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                invalid = true;
            }
            if (!Directory.Exists(path))
            {
                MessageBox.Show("The project path does not exist!", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                invalid = true;
            }
            if(!invalid)
            {
                DialogResult = DialogResult.OK;
                result = DialogResult.OK;
            }
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            DialogResult = DialogResult.Cancel;
			result = DialogResult.Cancel;
        }

        private void Intro_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (e.CloseReason == CloseReason.UserClosing)
            {
                result = DialogResult.Cancel;
				this.Close();
                //Environment.Exit(0);
            }
        }

        private void btnBrowse_Click(object sender, EventArgs e)
        {
            FolderBrowserDialog folderBrowserDialog1 = new FolderBrowserDialog();
            folderBrowserDialog1.SelectedPath = Environment.CurrentDirectory;
			
            
            if (folderBrowserDialog1.ShowDialog() == DialogResult.OK)
            {
                tbPath.Text = folderBrowserDialog1.SelectedPath;
            }
        }

    }
}
