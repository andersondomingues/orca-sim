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
    public partial class Intro : Form
    {
        string name;
        bool load;
        bool result; /* indicates HempsGenerator.cs if the user has selected the Cancel button */

        public Intro()
        {
            load = false;
            InitializeComponent();
        }

        public string NAme
        {
            get { return name; }
            set { name = value; }
        }

        public bool Restore
        {
            get { return load; }
            set { load = value; }
        }

        public bool Result
        {
            get { return result; }
            set { result = value; }
        }

        private void newProject_Click(object sender, EventArgs e)
        {
            NewProject np;

            np = new NewProject();
            np.ShowDialog();
            np.Dispose();
            DialogResult = DialogResult.OK;

            /* Se eu fechei a janela clicando, nao executar essa linha */
            if (np.DialogResult == DialogResult.OK)
            {
                name = np.NAme;
                result = true;
            }
            else
            {
                result = false;
            }
        }

        private void loadProject_Click(object sender, EventArgs e)
        {
            load = true;
            DialogResult = DialogResult.OK;
        }

        /*private void Intro_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (e.CloseReason == CloseReason.UserClosing)
            {
                Environment.Exit(0);
            }
        }*/
    }
}
