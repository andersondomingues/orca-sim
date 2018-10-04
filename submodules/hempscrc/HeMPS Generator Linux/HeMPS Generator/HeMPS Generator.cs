using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Diagnostics;
using System.IO;
using System.Globalization;

namespace HeMPS_Generator
{
	
    public partial class Generator : Form
    {

        private int kernelPages;                            /* Number of pages used by kernel */
        private const int KERNEL_SIZE = 30;                 /* Kernel size in KB (safety oversized)*/
        private ProcessingElement[,] processingElement;    /* MPSoC */
        private Repository[] repository;                    /* Repository for tasks to run in the master processor */
        private List<Task> lstAllocatedTasks;               /* Stores all allocated tasks after press the Generate HeMPS button */
        private String workingDirectory;                    /* Directory which HeMPS Generator.exe is being executed*/
		private String hemps_pathDirectory;                 /* Directory with HEMPS_PATH environment variable */
        private String projectName = null;                    /* Name of the current project */
		private String projectPath = null;					/* Path of the Current project ... */
        private Hashtable AppStartTime;                     /* Stores the start time of each application */
        private List<Task> TaskIDs;                         /* List used to store the tasks sorted by their IDs */
        List<taskList> tskList = new List<taskList>();      /* Store a list of tasks with it dependences */
		public bool projectIsSaved = false;
        int masterAddressGlobal = -1;
		
        public Generator()
        {

            /* Initialize the main window components */
            InitializeComponent();
			
			this.Size = new Size(1024, 768);
			this.MaximumSize = new Size(Screen.PrimaryScreen.WorkingArea.Width, Screen.PrimaryScreen.WorkingArea.Height);

            cbUnit.SelectedIndex = 2;

            workingDirectory = Environment.CurrentDirectory;
			
			hemps_pathDirectory = System.Environment.GetEnvironmentVariable("HEMPS_PATH");

            lstAllocatedTasks = new List<Task>();

            TaskIDs = new List<Task>();

            AppStartTime = new Hashtable();

            processingElement = new ProcessingElement[(int)nudX.Maximum, (int)nudY.Maximum];

            /* Creates the network nodes (processors) */
            for (int y = 0; y < (int)nudY.Maximum; y++)
                for (int x = 0; x < (int)nudX.Maximum; x++)
                {
                    processingElement[x, y] = new ProcessingElement();
                    processingElement[x, y].NewMaster += new EventHandler(ProcessingElement_NewMaster);
                }

            /* Creates the repository */
            repository = new Repository[2];

            /* Sets the kind of the repositorys */
            repository[0] = new Repository(0); /* Repository to the plasma tasks */
            repository[1] = new Repository(1); /* Repository to the mblite tasks */


            /* Sets the repository panel size */
            tlpRepository.RowCount = 2;
            tlpRepository.ColumnCount = 1;

            /* Adds the two repositories to the panel */
            tlpRepository.Controls.Add(repository[0], 0, 0);
            tlpRepository.Controls.Add(repository[1], 0, 1);

            /* Sets the initial processing elements panel size */
            tlpProcessingElements.RowCount = (int)nudY.Value;
            tlpProcessingElements.ColumnCount = (int)nudX.Value;

            /* Sets the initial master processor */
            processingElement[1, 1].Master = true;

            /* Adds processors in the processors panel (nodes_panel) */
            for (int y = 0; y < nudY.Value; y++)
                for (int x = 0; x < nudX.Value; x++)
                {
                    processingElement[x, y].X = x;
                    processingElement[x, y].Y = y;
                    processingElement[x, y].Address = NodeAddress(x, y);
                    tlpProcessingElements.Controls.Add(processingElement[x, y], x, (int)nudY.Value - y - 1);
                }

            /* Sets the initial page size */
            dudPageSize.SelectedItem = "16";

            /* Sets the initial duration unit */
            dudTime.SelectedItem = "ms";

            /* Intro screen */
            //Begin();



        }
		
		void runCommand(string command){
			runShellCommand("xterm", "-e \""+ command +"\"", 1);
		}
		
		// Funcao que executa um comando em shell
		// Se o WAIT tiver ativado a execusao espera ate o comando terminar
		protected virtual bool runShellCommand(string szCmd, string szArgs, int wait)
		{
			if( szCmd == null ) return false;
			System.Diagnostics.Process myproc = new System.Diagnostics.Process( );
			myproc.EnableRaisingEvents = false;
			myproc.StartInfo.FileName = szCmd;
			myproc.StartInfo.Arguments = szArgs;
			if(myproc.Start( ))
			{
				if( wait == 1 ) myproc.WaitForExit( );
				else myproc.Close( );
				return true;
			}
			else return false;
		}
		
        /* Generates an Hamiltonian address from a (x,y) coordinates */
        int NodeAddress(int x, int y)
        {
        	return (x << 4 ^ y);
        }

        /* Adds a new application in the application tree */
        private void btnAddApplication_Click(object sender, EventArgs e)
        {

            FolderBrowserDialog fbdApplication = new FolderBrowserDialog();
            DirectoryInfo diApplicationDirectory;
            TreeNode tnApplication;
			
			projectIsSaved = false;

            /* Initializes the folder browser dialog */
            fbdApplication.ShowNewFolderButton = false;
            fbdApplication.Description = "Select the application folder";
			fbdApplication.SelectedPath = hemps_pathDirectory + "/applications";

            /* Show the folder browser dialog */
            if (fbdApplication.ShowDialog() == DialogResult.OK)
            {
                diApplicationDirectory = new DirectoryInfo(fbdApplication.SelectedPath);

                /* Verifies if the application has already been added */
                foreach (TreeNode tn in tvApplications.Nodes)
                    if (diApplicationDirectory.Name.Equals(tn.Text))
                    {
                        MessageBox.Show("You already have added this application.", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        return;
                    }

                tnApplication = new TreeNode(diApplicationDirectory.Name);

				/* Add the tasks to the application */
                foreach (FileInfo fi in diApplicationDirectory.GetFiles())
                    if (fi.Name.EndsWith(".c") || fi.Name.EndsWith(".cpp"))
                        tnApplication.Nodes.Add(fi.Name);

                /* Stores the application path (used in the makefile generation)*/
                tnApplication.ToolTipText = fbdApplication.SelectedPath;

                /* Adds the application in the application tree view*/
                tvApplications.Nodes.Add(tnApplication);
                tnApplication.ExpandAll();

                ListViewItem lvi = new ListViewItem(tnApplication.Text);
                lvi.SubItems.Add("1 ms");
                lvAppStartTime.Items.Add(lvi);
                AppStartTime.Add(tnApplication.Text, "1 ms");
            }
        }

        /* Removes the selected application from the application tree and its tasks from the processors */
        private void btnDeleteApplication_Click(object sender, EventArgs e)
        {

            if (tvApplications.SelectedNode != null && tvApplications.SelectedNode.Level == 0)
            {

                /*  Removes the application tasks from the processors */
                foreach (ProcessingElement pe in processingElement)
                    pe.RemoveApplicationTasks(tvApplications.SelectedNode);

                foreach (Repository rep in repository)
                    rep.RemoveApplicationTasks(tvApplications.SelectedNode);

                AppStartTime.Remove(tvApplications.SelectedNode.Text);

                foreach (ListViewItem lvi in lvAppStartTime.Items)
                    if (lvi.Text == tvApplications.SelectedNode.Text) lvAppStartTime.Items.Remove(lvi);

                lvAppStartTime.Update();

                tvApplications.Nodes.Remove(tvApplications.SelectedNode);
            }
        }

        /* Removes all allocated tasks */
        private void btnResetAllocation_Click(object sender, EventArgs e)
        {

            foreach (ProcessingElement pe in tlpProcessingElements.Controls)
                pe.RemoveAllTasks();
            foreach (Repository rep in tlpRepository.Controls)
                rep.RemoveAllTasks();
        }

        /* Adds/Removes columns in the processors window (table layout panel) */
        private void nudX_ValueChanged(object sender, EventArgs e)
        {

            /* Avoids the dimension 1x1 */
            if (nudX.Value == 1 && nudY.Value == 1)
            {
                nudX.Value = 2;
                return;
            }

            /* removes the application in the column that will be deleted */
            if (tlpProcessingElements.ColumnCount > (int)nudX.Value)
                for (int y = 0; y < tlpProcessingElements.RowCount; y++)
                    processingElement[tlpProcessingElements.ColumnCount - 1, y].RemoveAllTasks();

            tlpProcessingElements.SuspendLayout();

            tlpProcessingElements.ColumnCount = (int)nudX.Value;

            tlpProcessingElements.Controls.Clear();

            /*** Add processors in the window ***/
            for (int y = 0; y < nudY.Value; y++)
                for (int x = 0; x < nudX.Value; x++)
                {
                    processingElement[x, y].X = x;
                    processingElement[x, y].Y = y;
                    processingElement[x, y].Address = NodeAddress(x, y);
                    tlpProcessingElements.Controls.Add(processingElement[x, y], x, (int)nudY.Value - y - 1);
                }

            tlpProcessingElements.ResumeLayout();
			projectIsSaved = false;
        }

        /* Adds/Removes rows in the processors window (table layout panel) */
        private void nudY_ValueChanged(object sender, EventArgs e)
        {

            /* Avoids the dimension 1x1 */
            if (nudX.Value == 1 && nudY.Value == 1)
            {
                nudY.Value = 2;
                return;
            }

            /* removes the application in the row that will be deleted */
            if (tlpProcessingElements.RowCount > (int)nudY.Value)
                for (int x = 0; x < tlpProcessingElements.ColumnCount; x++)
                    processingElement[x, tlpProcessingElements.RowCount - 1].RemoveAllTasks();

            tlpProcessingElements.SuspendLayout();

            tlpProcessingElements.RowCount = (int)nudY.Value;

            tlpProcessingElements.Controls.Clear();

            /*** Add processors in the window ***/
            for (int y = 0; y < nudY.Value; y++)
                for (int x = 0; x < nudX.Value; x++)
                {
                    processingElement[x, y].X = x;
                    processingElement[x, y].Y = y;
                    processingElement[x, y].Address = NodeAddress(x, y);
                    tlpProcessingElements.Controls.Add(processingElement[x, y], x, (int)nudY.Value - y - 1);
                }

            tlpProcessingElements.ResumeLayout();
			projectIsSaved = false;
        }

        /* Recalculates the maximum number of tasks/slave when Memory Size or Page Size are updated */
        private void dudPageSize_SelectedItemChanged(object sender, EventArgs e)
        {

            int memorySize, pageSize, maxTasksSlave;

            if (rbtn64KB.Checked)
                memorySize = 64;
            else
                memorySize = 128;

            pageSize = Convert.ToUInt16(dudPageSize.SelectedItem.ToString());

            /* Calculates the kernel pages */
            if ((KERNEL_SIZE % pageSize) == 0)
                kernelPages = KERNEL_SIZE / pageSize;
            else
                kernelPages = KERNEL_SIZE / pageSize + 1;

            maxTasksSlave = memorySize / pageSize - kernelPages;

            tbxMaxTasksSlave.Text = Convert.ToString(maxTasksSlave);

            /* Updates the maximum tasks/slaves in all processors */
            foreach (ProcessingElement pe in processingElement)
            {
                pe.MaximumTasks = maxTasksSlave;
                pe.RemoveExtraTasks();
            }
			projectIsSaved = false;
        }

        /* Generates files: makefile, applications IDs, ids_master.h, ids_slave.h, HeMPSGeneratorPackage.vhd */
        private void btnGenerate_Click(object sender, EventArgs e)
        {
			if(projectPath==null) 
			{
				SaveAs();
				projectIsSaved = true;
			}
			if(projectPath==null) 
			{
				MessageBox.Show("You must create a project directory!", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
			}
            
			TreeNode tnApplication;
            StreamWriter makefile, idFile;
            List<Task> ProjectTimeInsertedTasks = new List<Task>();
            List<Task> RunTimeInsertedTasks = new List<Task>();
            Hashtable RunTimeInsertedAppsTasks = new Hashtable();
            Hashtable AppsInitTasks = new Hashtable();
            String TimeAux;
            int masterAddress = -1;
            int masterAddress_ids_slave = -1;
            int slaveProcessors = 0;	    /* Count the enabled slave processors */
            int plasmaCoreProcessors = 0;   /* Count the plasma slave processors
                                             * MBLite core processors = slaveProcessors - plasmaCoreProcessors */

            lstAllocatedTasks.Clear();
            TaskIDs.Clear();
			/* Search all processors */
            foreach (ProcessingElement pe in tlpProcessingElements.Controls)
            {

                /* Looks for the master processor */
                if (pe.Master)
                {
                    masterAddress = pe.Y * (int)nudX.Value + pe.X;
                    masterAddressGlobal=masterAddress;
					masterAddress_ids_slave = pe.X * 10 + pe.Y;
                }

                /* Count the number of enabled processors and the number of Plasma Processors*/
                if (pe.Slave)
                {
                    slaveProcessors++;

                    if (pe.Plasma)
                        plasmaCoreProcessors++;
                }

                /* Retrieves all allocated tasks */
                foreach (Task task in pe.TaskList.Items)
                {
                    lstAllocatedTasks.Add(task); /* List used in the application IDs and ids_kernel generation */
                    if (pe.Plasma)
                        task.CoreType = 0;
                    else
                        task.CoreType = 1;
                }

            }

            /* Searches the repositories for allocated tasks */
            foreach (Repository rep in tlpRepository.Controls)
                foreach (Task t in rep.TaskList.Items)
                {
                    lstAllocatedTasks.Add(t);
                    if (rep.Repository_Type == 0)
                        t.CoreType = 0;
                    else
                        t.CoreType = 1;
                }

            List<KeyValuePair<TreeNode, float>> SortedApps = new List<KeyValuePair<TreeNode, float>>();

            CultureInfo point = (CultureInfo)CultureInfo.CurrentCulture.Clone();
            point.NumberFormat.CurrencyDecimalSeparator = ".";

            CultureInfo comma = (CultureInfo)CultureInfo.CurrentCulture.Clone();
            comma.NumberFormat.CurrencyDecimalSeparator = ",";

            foreach (TreeNode tn in tvApplications.Nodes)
            {
                TimeAux = AppStartTime[tn.Text].ToString();
                float apptime = 0;
                if (TimeAux.Contains(".")) apptime = float.Parse(TimeAux.Split(' ')[0], NumberStyles.Any, point);
                if (TimeAux.Contains(",")) apptime = float.Parse(TimeAux.Split(' ')[0], NumberStyles.Any, comma);
                if (!TimeAux.Contains(".") && !TimeAux.Contains(",")) apptime = float.Parse(TimeAux.Split(' ')[0]);
                String unity = TimeAux.Split(' ')[1];

                if (apptime != 0)
                {
                    if (unity == "ns") apptime = apptime / 1000000;
                    if (unity == "us") apptime = apptime / 1000;
                    if (unity == "s") apptime = apptime * 1000;
                }
                SortedApps.Add(new KeyValuePair<TreeNode, float>(tn, apptime));
            }

            SortedApps.Sort(
                delegate(KeyValuePair<TreeNode, float> val1,
                KeyValuePair<TreeNode, float> val2)
                {
                    return val1.Value.CompareTo(val2.Value);
                }
            );

            /*  Gets all applications */
            foreach (KeyValuePair<TreeNode, float> element in SortedApps)
            {
                TreeNode tn = (TreeNode)element.Key;
				/* Searches allocated task belonging to an applications */
                foreach (Task task in lstAllocatedTasks)
                {
                    if (object.ReferenceEquals(task.Application, tn))
                    {
                        if ((float)element.Value == 0) ProjectTimeInsertedTasks.Add(task);
                        else
                        {
                            RunTimeInsertedTasks.Add(task);
                            if (!RunTimeInsertedAppsTasks.Contains(tn.Text))
                            {
                                List<Task> AppTasks = new List<Task>();
                                AppTasks.Add(task);
                                RunTimeInsertedAppsTasks.Add(tn.Text, AppTasks);
						    }
                            else
                            {
                                List<Task> AppTasks = (List<Task>)RunTimeInsertedAppsTasks[tn.Text];
                                AppTasks.Add(task);
                                RunTimeInsertedAppsTasks[tn.Text] = AppTasks;
						    }
                        }
                    }
                }
            }
            TaskIDs.AddRange(ProjectTimeInsertedTasks);
            TaskIDs.AddRange(RunTimeInsertedTasks);
			
			
			#region Information Messages

            if (masterAddress == -1)
            { /* No master processor */
                MessageBox.Show("There is no master processor.\nSelect a processor as master before continue.", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            else if ((rbtnSc.Checked || rbtnIss.Checked || rbtnScModelsim.Checked) && (slaveProcessors != plasmaCoreProcessors)) /* ISS and SC does not support heterogeneous architectures */
            {
                MessageBox.Show("ISS and SC processor descriptions does not support heterogeneous architectures.", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            else if (tvApplications.Nodes.Count == 0)
            { /* No applications */
                MessageBox.Show("There is none application added.\nAdd an application before continue.", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }            
			else if (lstAllocatedTasks.Count == 0)	
            { /* No tasks allocated */
                MessageBox.Show("There are no allocated tasks.\nAt least one task must be allocated.", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            else if ((repository[1].TaskList.Items.Count > 0) && (slaveProcessors == plasmaCoreProcessors))
            { /* There's no mblite processor and there is mblite tasks */
                MessageBox.Show("There are no MBLite processors, though there is tasks to run in MBLite processors.", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            else if ((repository[0].TaskList.Items.Count > 0) && (plasmaCoreProcessors == 0))
            { /* there is no plasma processor and there is plasma tasks */
                MessageBox.Show("There are no Plasma processors, though there is tasks to run in Plasma processors.", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            /* Verifies if there is some tasks not allocated */
            else
            {
                foreach (TreeNode tn in tvApplications.Nodes)
                    if (tn.Nodes.Count > 0)
                    {
                        MessageBox.Show("There are some tasks not allocated.\nAll tasks must be allocated before continuing.", "Information", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        return;
                    }
            }

            #endregion
			
			
			if(projectIsSaved == false){
				Save(false);
			}
			
            this.Cursor = Cursors.WaitCursor;   /* Changes the cursor type during the generation process */
            btnGenerate.Enabled = false;        /* Disable the Generation button during the generation process */

            progressBar.Value += 5;
            lblProgress.Text = "Generating HeMPS_PKG.vhd";

			
			
			
			//MessageBox.Show(System.Environment.GetEnvironmentVariable("HEMPS_PATH") );
			
            /* Generates the HeMPSGeneratorPackage.vhd */
            #region HeMPS_PKG

            int memorySize, memSize, maxTasksSlave, pageSize, page_size_h_index, page_number_h_index;
            StringBuilder core = new StringBuilder();

            /* calculates the necessary information to define the values */
            pageSize = Convert.ToUInt16(dudPageSize.SelectedItem.ToString());

            /* calculates the memory size */
            if (rbtn64KB.Checked)
                memorySize = 64;
            else
                memorySize = 128;

            memSize = memorySize * 1024;

            /* Calculates the kernel pages */
            if ((KERNEL_SIZE % pageSize) == 0)
                kernelPages = KERNEL_SIZE / pageSize;
            else
                kernelPages = KERNEL_SIZE / pageSize + 1;

            /* defines the maximum tasks/slave */
            maxTasksSlave = memorySize / pageSize - kernelPages;

            /* gets the page_size_h_index */
            if (pageSize == 32)
                page_size_h_index = 14;
            else
                page_size_h_index = 13;

            /* gets the page_number_h_index */
            if (maxTasksSlave < 4)
                page_number_h_index = page_size_h_index + 2;
            else
                page_number_h_index = page_size_h_index + 3;

            System.Text.StringBuilder HempsPackage = new System.Text.StringBuilder();
            String line;
            String aux;
            StreamWriter HeMPSGeneratorPackage_vhd_wr;
            String[] split;

            /* Reads/Edits the file HeMPS_PKG.vhd */
            try
            {

                HempsPackage.AppendLine();
                HempsPackage.AppendLine("--------------------------------------------------------------------------");
                HempsPackage.AppendLine("-- package com tipos basicos");
                HempsPackage.AppendLine("--------------------------------------------------------------------------");
                HempsPackage.AppendLine("library IEEE;");
                HempsPackage.AppendLine("use IEEE.Std_Logic_1164.all;");
                HempsPackage.AppendLine("use IEEE.std_logic_unsigned.all;");
                HempsPackage.AppendLine("use IEEE.std_logic_arith.all;");
                HempsPackage.AppendLine();
                HempsPackage.AppendLine("package HeMPS_PKG is");
                HempsPackage.AppendLine();
                HempsPackage.AppendLine("--------------------------------------------------------");
                HempsPackage.AppendLine("-- HEMPS CONSTANTS");
                HempsPackage.AppendLine("--------------------------------------------------------");
                HempsPackage.AppendLine("\t-- paging definitions");
                HempsPackage.AppendLine("\tconstant PAGE_SIZE_H_INDEX\t\t: integer := " + page_size_h_index + ";");
                HempsPackage.AppendLine("\tconstant PAGE_NUMBER_H_INDEX\t: integer := " + page_number_h_index + ";");
                HempsPackage.AppendLine();
                HempsPackage.AppendLine("\t-- Hemps top definitions");
                HempsPackage.AppendLine("\tconstant NUMBER_PROCESSORS_X\t: integer := " + nudX.Value + "; ");
                HempsPackage.AppendLine("\tconstant NUMBER_PROCESSORS_Y\t: integer := " + nudY.Value + "; ");
                HempsPackage.AppendLine();
                HempsPackage.AppendLine("\tconstant MASTER_ADDRESS\t\t\t: integer := " + masterAddress + ";");
                HempsPackage.AppendLine("\tconstant NUMBER_PROCESSORS\t\t: integer := NUMBER_PROCESSORS_Y*NUMBER_PROCESSORS_X;");
                HempsPackage.AppendLine();
                HempsPackage.AppendLine("\tconstant NUMBER_OF_APPS\t\t\t: integer := " + RunTimeInsertedAppsTasks.Count + ";");
                if (RunTimeInsertedAppsTasks.Count != 0)
                {
                    aux = "";
                    float pastTime = 0;
                    float delay = 0;
                    for (int i = 0; i < SortedApps.Count; i++)
                    {
                        if (SortedApps[i].Value != 0)
                        {
                            delay = SortedApps[i].Value - pastTime;
                            aux = aux + delay.ToString().Replace(',', '.') + " ms,";
                            pastTime = SortedApps[i].Value;
                        }
                    }
                    aux = "(" + aux + "0 ms);";
                    HempsPackage.AppendLine("\ttype timearray is array(0 to NUMBER_OF_APPS) of time;");
                    HempsPackage.AppendLine("\tconstant appstime : timearray := " + aux);
                }
                else
                {
                    HempsPackage.AppendLine("\ttype timearray is array(0 to 1) of time;");
                    HempsPackage.AppendLine("\tconstant appstime : timearray := (0 ms, 0 ms);");
                }
                HempsPackage.AppendLine();
                HempsPackage.AppendLine("\tsubtype core_str is string(1 to 6);");
                HempsPackage.AppendLine("\ttype core_type_type is array(0 to NUMBER_PROCESSORS-1) of core_str;");
                aux = "";
                foreach (ProcessingElement pe in tlpProcessingElements.Controls)
                {
                    if (pe.Plasma)
                        aux = aux + "\n\t\t\t\t\t\t\t\t\t\t\t\"plasma\",";
                    else
                        aux = aux + "\n\t\t\t\t\t\t\t\t\t\t\t\"mblite\",";
                }
                aux = "(" + aux + ");";
                string[] convert2 = new string[] { ",)" };
                /* Takes the last coma out of it */
                split = aux.Split(convert2, StringSplitOptions.None);
                split[0] = split[0] + ");";
                aux = split[0];
                HempsPackage.AppendLine("\tconstant core_type : core_type_type := " + aux);
                HempsPackage.AppendLine("end HeMPS_PKG;");


                /* Writes the new Hermes_package.vhd file */
                HeMPSGeneratorPackage_vhd_wr = new StreamWriter(projectPath + "//HeMPS_PKG.vhd");
                HeMPSGeneratorPackage_vhd_wr.Write(HempsPackage.ToString());
                HeMPSGeneratorPackage_vhd_wr.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion

            #region HeMPS_PKG_H

            System.Text.StringBuilder hemps_pkg_h = new System.Text.StringBuilder();
            StreamWriter hemps_pkg_h_wr;

            try
            {

                hemps_pkg_h.AppendLine("#define N_PE_X " + nudX.Value);
                hemps_pkg_h.AppendLine("#define N_PE_Y " + nudY.Value);
                hemps_pkg_h.AppendLine("#define N_PE N_PE_Y*N_PE_X");
                hemps_pkg_h.AppendLine("#define MASTER " + masterAddress);
				hemps_pkg_h.AppendLine("	");
				
                /* Writes the new Makefile file */
                hemps_pkg_h_wr = new StreamWriter(projectPath + "//HeMPS_PKG.h");
                hemps_pkg_h_wr.Write(hemps_pkg_h.ToString());
                hemps_pkg_h_wr.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }

            #endregion

            /* Cleans the build directory */
            Environment.CurrentDirectory = workingDirectory;

            progressBar.Value += 5;
            lblProgress.Text = "Generating makefile";

            /* Generates the makefile */

            #region Makefile

            StringBuilder tasks = new StringBuilder();
            StringBuilder all = new StringBuilder("all: ");
            string taskInclude, taskSourceFileName, taskName, applicationDirectory;


            int taskId;


            try
            {
                makefile = new StreamWriter(projectPath + "/build/makefile");

				makefile.WriteLine("#this environment variable must point to the hemps path, where the hardware, software and tools folders are located");
				//makefile.WriteLine("HEMPS_PATH = " + workingDirectory);
				makefile.WriteLine("");
                makefile.WriteLine("#Definition of Plasma toolchain");
                makefile.WriteLine("CFLAGS     = -O2 -Wall -c -s");
                makefile.WriteLine("GCC_MIPS   = mips-elf-gcc $(CFLAGS)");
                makefile.WriteLine("AS_MIPS    = mips-elf-as");
                makefile.WriteLine("LD_MIPS    = mips-elf-ld");
                makefile.WriteLine("DUMP_MIPS  = mips-elf-objdump");
				makefile.WriteLine("COPY_MIPS = mips-elf-objcopy -I elf32-bigmips -O binary\n");

				
  				makefile.WriteLine("#Definition of MB-Lite toolchain");
				makefile.WriteLine("MB         = mb-gcc");
                makefile.WriteLine("AS         = mb-as");
                makefile.WriteLine("LD         = mb-ld");
                makefile.WriteLine("MB_OBJCOPY = mb-objcopy");
                makefile.WriteLine("MB_OBJDUMP = mb-objdump");

                makefile.WriteLine("XILFLAGS   =-mxl-soft-div -msoft-float -mxl-barrel-shift -mno-xl-soft-mul");
                makefile.WriteLine("CXXFLAGS   =-g -std=c99 -pedantic -Wall -O2 ");
                makefile.WriteLine("LNKFLAGS	   =-Wl,-defsym -Wl,_STACK_SIZE=0x3000 -Wl,-defsym -Wl,_HEAP_SIZE=0x0000");
                makefile.WriteLine("LNKFLAGS2  =-Wl,-defsym -Wl,_STACK_SIZE=0x2000 -Wl,-defsym -Wl,_HEAP_SIZE=0x0000");

                makefile.WriteLine("MB_GCC     = $(MB) $(XILFLAGS) $(CXXFLAGS) $(LNKFLAGS2) $(LIBFLAGS) $(INCFLAGS) $(CCFLAGS)");

                makefile.WriteLine();

                makefile.WriteLine("#TOOLS");
                makefile.WriteLine("BIN2MEM       = bin2mem");
                makefile.WriteLine("CONVERT       = -gatomaker");
                makefile.WriteLine("RAM_GENERATOR = ram_generator");
                makefile.WriteLine();

                makefile.WriteLine("INCLUDE       = $(HEMPS_PATH)/software/include");
                makefile.WriteLine();

                makefile.WriteLine("#TASKS");

                int num_tasks = 0;


                foreach (Task task in TaskIDs)
                {

                    taskSourceFileName = task.Text;
                    num_tasks = TaskIDs.IndexOf(task);

                    /* sets the task path */
                    tnApplication = task.Application;
                    applicationDirectory = tnApplication.ToolTipText;
					//MessageBox.Show("../applications/" + tnApplication.Text + "/" + taskSourceFileName);
                    makefile.WriteLine("TASK" + num_tasks + "_PATH = " + "../applications/" + tnApplication.Text + "/" + taskSourceFileName);

                    /* sets the task include files */
                    taskInclude = "ids_" + tnApplication.Text + ".h";
                    makefile.WriteLine("TASK" + num_tasks + "_INCLUDE = " + taskInclude);

                    /* sets the task source name */
                    taskName = taskSourceFileName.Remove(taskSourceFileName.IndexOf('.'));
                    makefile.WriteLine("TASK" + num_tasks + "_NAME = " + taskName);

                    /* sets the task ID */
                    taskId = TaskIDs.IndexOf(task);
                    makefile.WriteLine("TASK" + num_tasks + "_ID = " + taskId);

                    /* sets the task make target */
                    makefile.WriteLine("TASK" + num_tasks + "_TARGET = $(TASK" + num_tasks + "_NAME)_$(TASK" + num_tasks + "_ID)");

                    makefile.WriteLine();

                }

				
				makefile.WriteLine("#tasks boot code for Plasma processor");
                makefile.WriteLine("BOOT_TASK_SRC     = $(HEMPS_PATH)/software/include/bootTask.asm");
				makefile.WriteLine("BOOT_TASK         = bootTask");
                makefile.WriteLine("#kernel master source files");
				makefile.WriteLine("BOOT_MASTER_SRC   = $(HEMPS_PATH)/software/kernel/master/boot.S");
				makefile.WriteLine("BOOT_MASTER       = boot_master");
                makefile.WriteLine("KERNEL_MASTER_SRC = $(HEMPS_PATH)/software/kernel/master/kernel.c");
				makefile.WriteLine("KERNEL_MASTER     = kernel_master");
                makefile.WriteLine("#kernel slave plasma source files");
				makefile.WriteLine("BOOT_PLASMA_SRC   = $(HEMPS_PATH)/software/kernel/slave/boot.S");
				makefile.WriteLine("BOOT_PLASMA       = boot_plasma");
                makefile.WriteLine("KERNEL_PLASMA_SRC = $(HEMPS_PATH)/software/kernel/slave/kernel.c");
				makefile.WriteLine("KERNEL_PLASMA     = kernel_plasma");
				makefile.WriteLine("#kernel slave mblite source files");
				makefile.WriteLine("KERNEL_MBLITE     = kernel_mblite");
                makefile.WriteLine("INTERRUPT_SRC     = $(HEMPS_PATH)/software/kernel/slave/interrupt.s");
				makefile.WriteLine("INTERRUPT         = interrupt");
                makefile.WriteLine();


                /* Task boot make target */
                makefile.WriteLine("bootTask:");
                makefile.WriteLine("\t$(AS_MIPS) -o $(BOOT_TASK).o $(BOOT_TASK_SRC)");
                makefile.WriteLine();
                all.Append("bootTask ");

                /* Kernel master make target */
                makefile.WriteLine("kernel_master:");
                makefile.WriteLine("\t$(AS_MIPS) -o $(BOOT_MASTER).o $(BOOT_MASTER_SRC)");
                makefile.WriteLine("\t$(GCC_MIPS) -o $(KERNEL_MASTER).o $(KERNEL_MASTER_SRC) --include ids_master.h --include InitializeVectorsMaster.h");
                makefile.WriteLine("\t$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_MASTER).map -s -N -o $(KERNEL_MASTER).bin  $(BOOT_MASTER).o $(KERNEL_MASTER).o");
                makefile.WriteLine("\t$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_MASTER)_debug.map -o $(KERNEL_MASTER)_debug.bin  $(BOOT_MASTER).o $(KERNEL_MASTER).o");
                makefile.WriteLine("\t$(DUMP_MIPS) -S $(KERNEL_MASTER)_debug.bin > $(KERNEL_MASTER).lst");
                makefile.WriteLine("\t$(COPY_MIPS) $(KERNEL_MASTER).bin $(KERNEL_MASTER).dump");
				makefile.WriteLine("\thexdump -v -e '1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" \"\\n\"' $(KERNEL_MASTER).dump > $(KERNEL_MASTER).txt");
				makefile.WriteLine("\tsed -i -e '4s/.*/37bdc800/' $(KERNEL_MASTER).txt");
                makefile.WriteLine();
                all.Append("kernel_master ");


                /* Kernel slave make target - Plasma */
                makefile.WriteLine("kernel_plasma:");
                makefile.WriteLine("\t$(AS_MIPS) -o $(BOOT_PLASMA).o $(BOOT_PLASMA_SRC)");
                makefile.WriteLine("\t$(GCC_MIPS) -o $(KERNEL_PLASMA).o $(KERNEL_PLASMA_SRC) --include ids_slave.h --include InitializeVectorsSlave.h -D PLASMA");
                makefile.WriteLine("\t$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_PLASMA).map -s -N -o $(KERNEL_PLASMA).bin  $(BOOT_PLASMA).o $(KERNEL_PLASMA).o");
                makefile.WriteLine("\t$(LD_MIPS) -Ttext 0 -eentry -Map $(KERNEL_PLASMA)_debug.map -o $(KERNEL_PLASMA)_debug.bin  $(BOOT_PLASMA).o $(KERNEL_PLASMA).o");
                makefile.WriteLine("\t$(DUMP_MIPS) -S $(KERNEL_PLASMA)_debug.bin > $(KERNEL_PLASMA).lst");
                makefile.WriteLine("\t$(COPY_MIPS) $(KERNEL_PLASMA).bin $(KERNEL_PLASMA).dump");
				makefile.WriteLine("\thexdump -v -e '1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" \"\\n\"' $(KERNEL_PLASMA).dump > $(KERNEL_PLASMA).txt");
				makefile.WriteLine("\tsed -i -e '4s/.*/37bd7800/' $(KERNEL_PLASMA).txt");
                makefile.WriteLine();
                all.Append("kernel_plasma ");

				
		
	            /* Kernel slave make target - MBLite */
	            makefile.WriteLine("kernel_mblite:");
				if((slaveProcessors - plasmaCoreProcessors) > 0){
	                makefile.WriteLine("\t$(AS) -o $(INTERRUPT).o $(INTERRUPT_SRC)");
	                makefile.WriteLine("\t$(MB_GCC) $(KERNEL_PLASMA_SRC) $(INTERRUPT).o -o $(KERNEL_MBLITE).elf --include ids_slave.h --include InitializeVectorsSlave.h -D MBLITE");
	                makefile.WriteLine("\t$(MB_OBJCOPY) -O binary $(KERNEL_MBLITE).elf $(KERNEL_MBLITE).bin");
	                makefile.WriteLine("\t$(MB_OBJDUMP) -DSCz $(KERNEL_MBLITE).elf > $(KERNEL_MBLITE).dump");
	                makefile.WriteLine("\t$(BIN2MEM) $(KERNEL_MBLITE).bin > $(KERNEL_MBLITE).txt");
	                //makefile.WriteLine("\t-$(CONVERT) $(KERNEL_MBLITE).dump $(KERNEL_MBLITE).mem > $(KERNEL_MBLITE).txt");
					makefile.WriteLine("\tgrep '<puta>' $(KERNEL_MBLITE).dump | sed 's/^....\\(....\\).*$$/\\1/' | xargs -I {} sed -i -e '5s/.*/b808{}/' $(KERNEL_MBLITE).txt");	
	                makefile.WriteLine();
				}else{
					makefile.WriteLine("\t@echo \"generating dummy kernel for mblite processor\"");
					makefile.WriteLine("\tcp $(KERNEL_MASTER).txt $(KERNEL_MBLITE).txt");
	                makefile.WriteLine();
				}
				
	            all.Append("kernel_mblite ");

                /*  Generate the tasks make targets - tasks in processors */
                foreach (ProcessingElement pe in tlpProcessingElements.Controls)
                {
                    if (pe.TaskList.Items.Count > 0)
                    {

                        foreach (Task task in pe.TaskList.Items)
                        {

                            /* Retrives the parent application, which the task belongs */
                            tnApplication = task.Application;
                            taskInclude = "ids_" + tnApplication.Text + ".h";

                            taskSourceFileName = task.Text;
                            taskName = taskSourceFileName.Remove(taskSourceFileName.IndexOf('.'));
                            taskName = taskName + "_" + TaskIDs.IndexOf(task);
                            tasks.Append(taskName + ".txt ");	/* Used in the rom_loader make target */
                            all.Append(taskName + " ");
                            applicationDirectory = tnApplication.ToolTipText;
                            num_tasks = TaskIDs.IndexOf(task);

                            /* Task make target */
                            makefile.WriteLine(taskName + ":");

                            if (pe.Plasma)
                            {
                                makefile.WriteLine("\t$(GCC_MIPS) $(TASK" + num_tasks + "_PATH) -o $(TASK" + num_tasks + "_TARGET).o --include $(TASK" + num_tasks + "_INCLUDE)" + " -D PLASMA -I $(INCLUDE)");
                                makefile.WriteLine("\t$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK" + num_tasks + "_TARGET).map " + "-s -N -o $(TASK" + num_tasks + "_TARGET).bin " + "$(BOOT_TASK).o $(TASK" + num_tasks + "_TARGET).o");
                                makefile.WriteLine("\t$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK" + num_tasks + "_TARGET)_debug.map " + "-o $(TASK" + num_tasks + "_TARGET)_debug.bin " + "$(BOOT_TASK).o $(TASK" + num_tasks + "_TARGET).o");
                                makefile.WriteLine("\t$(DUMP_MIPS) -S $(TASK" + num_tasks + "_TARGET)_debug.bin > $(TASK" + num_tasks + "_TARGET).lst");
                                makefile.WriteLine("\t$(COPY_MIPS) $(TASK" + num_tasks + "_TARGET).bin $(TASK" + num_tasks + "_TARGET).dump");
								makefile.WriteLine("\thexdump -v -e '1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" \"\\n\"' $(TASK" + num_tasks + "_TARGET).dump > $(TASK" + num_tasks + "_TARGET).txt");
								makefile.WriteLine("\tsed -i -e '4s/.*/37bd4000/' $(TASK" + num_tasks + "_TARGET).txt");
                                makefile.WriteLine();
                            }
                            else
                            {

                                makefile.WriteLine("\t$(MB_GCC) $(TASK" + num_tasks + "_PATH) -o $(TASK" + num_tasks + "_TARGET).elf --include $(TASK" + num_tasks + "_INCLUDE) -D MBLITE -I $(INCLUDE)");
                                makefile.WriteLine("\t$(MB_OBJCOPY) -O binary $(TASK" + num_tasks + "_TARGET).elf $(TASK" + num_tasks + "_TARGET).bin");
                                makefile.WriteLine("\t$(MB_OBJDUMP) -DSCz $(TASK" + num_tasks + "_TARGET).elf > $(TASK" + num_tasks + "_TARGET).lst");
                                makefile.WriteLine("\t$(BIN2MEM) $(TASK" + num_tasks + "_TARGET).bin > $(TASK" + num_tasks + "_TARGET).txt");
                                makefile.WriteLine();
                            }
                        }
                    }
                }

                /*  Generate the tasks make targets - tasks in the repositories */
                foreach (Repository rep in tlpRepository.Controls)
                {
                    if (rep.TaskList.Items.Count > 0)
                    {

                        foreach (Task task in rep.TaskList.Items)
                        {

                            /* Retrives the parent application, which the task belongs */
                            tnApplication = task.Application;
                            taskInclude = "ids_" + tnApplication.Text + ".h";

                            taskSourceFileName = task.Text;
                            taskName = taskSourceFileName.Remove(taskSourceFileName.IndexOf('.'));
                            taskName = taskName + "_" + TaskIDs.IndexOf(task);
                            tasks.Append(taskName + ".txt ");	/* Used in the rom_loader make target */
                            all.Append(taskName + " ");
                            applicationDirectory = tnApplication.ToolTipText;
                            num_tasks = TaskIDs.IndexOf(task);

                            /* Task make target */
                            makefile.WriteLine(taskName + ":");

                            if (rep.Repository_Type == 0) /* Plasma repository */
                            {
                                makefile.WriteLine("\t$(GCC_MIPS) $(TASK" + num_tasks + "_PATH) -o $(TASK" + num_tasks + "_TARGET).o --include $(TASK" + num_tasks + "_INCLUDE)" + " -D PLASMA -I $(INCLUDE)");
                                makefile.WriteLine("\t$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK" + num_tasks + "_TARGET).map " + "-s -N -o $(TASK" + num_tasks + "_TARGET).bin " + "$(BOOT_TASK).o $(TASK" + num_tasks + "_TARGET).o");
                                makefile.WriteLine("\t$(LD_MIPS) -Ttext 0 -eentry -Map $(TASK" + num_tasks + "_TARGET)_debug.map " + "-o $(TASK" + num_tasks + "_TARGET)_debug.bin " + "$(BOOT_TASK).o $(TASK" + num_tasks + "_TARGET).o");
                                makefile.WriteLine("\t$(DUMP_MIPS) -S $(TASK" + num_tasks + "_TARGET)_debug.bin > $(TASK" + num_tasks + "_TARGET).lst");
                                makefile.WriteLine("\t$(COPY_MIPS) $(TASK" + num_tasks + "_TARGET).bin $(TASK" + num_tasks + "_TARGET).dump");
								makefile.WriteLine("\thexdump -v -e '1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" 1/1 \"%02x\" \"\\n\"' $(TASK" + num_tasks + "_TARGET).dump > $(TASK" + num_tasks + "_TARGET).txt");
								makefile.WriteLine("\tsed -i -e '4s/.*/37bd4000/' $(TASK" + num_tasks + "_TARGET).txt");
                                makefile.WriteLine();

                            }
                            else                          /* MBLite repository */
                            {

                                makefile.WriteLine("\t$(MB_GCC) $(TASK" + num_tasks + "_PATH) -o $(TASK" + num_tasks + "_TARGET).elf --include $(TASK" + num_tasks + "_INCLUDE) -D MBLITE -I $(INCLUDE)");
                                makefile.WriteLine("\t$(MB_OBJCOPY) -O binary $(TASK" + num_tasks + "_TARGET).elf $(TASK" + num_tasks + "_TARGET).bin");
                                makefile.WriteLine("\t$(MB_OBJDUMP) -DSCz $(TASK" + num_tasks + "_TARGET).elf > $(TASK" + num_tasks + "_TARGET).lst");
                                makefile.WriteLine("\t$(BIN2MEM) $(TASK" + num_tasks + "_TARGET).bin > $(TASK" + num_tasks + "_TARGET).txt");
                                makefile.WriteLine();
                            }
                        }
                    }
                }

                makefile.WriteLine("ram_gen: ram_master ram_plasma ram_mblite");
				makefile.WriteLine("\t");
				makefile.WriteLine("ram_master:");
                makefile.WriteLine("\t$(RAM_GENERATOR) -64 -vhd kernel_master.txt > ram_master.vhd");
                makefile.WriteLine("\tcp ram_master.vhd ../plasma_ram/rtl");
                makefile.WriteLine("\t$(RAM_GENERATOR) -64 -h kernel_master.txt > ram_master.h");
                makefile.WriteLine("\tcp ram_master.h ../plasma_ram/sc");
	            makefile.WriteLine();
				makefile.WriteLine("ram_plasma:");
                makefile.WriteLine("\t$(RAM_GENERATOR) -64 -vhd kernel_plasma.txt > ram_plasma.vhd");
                makefile.WriteLine("\tcp ram_plasma.vhd ../plasma_ram/rtl");
                makefile.WriteLine("\t$(RAM_GENERATOR) -64 -h kernel_plasma.txt > ram_plasma.h");
                makefile.WriteLine("\tcp ram_plasma.h ../plasma_ram/sc");
	            makefile.WriteLine();
				makefile.WriteLine("ram_mblite:");
                makefile.WriteLine("\t$(RAM_GENERATOR) -64 -vhd kernel_mblite.txt > ram_mblite.vhd");
                makefile.WriteLine("\tcp ram_mblite.vhd ../mblite_ram/rtl");
                makefile.WriteLine("\t$(RAM_GENERATOR) -64 -h kernel_mblite.txt > ram_mblite.h");
                makefile.WriteLine("\tcp ram_mblite.h ../mblite_ram/sc");
                makefile.WriteLine();
                all.Append("ram_gen");
				
				makefile.WriteLine("clean:");
				makefile.WriteLine("\trm -rf *.bin");
				makefile.WriteLine("\trm -rf *.txt");
				makefile.WriteLine("\trm -rf *.mem");
				makefile.WriteLine("\trm -rf *.dump");
				makefile.WriteLine("\trm -rf *.lst");
				makefile.WriteLine("\trm -rf *.o");
				makefile.WriteLine("\trm -rf *.map");
				makefile.WriteLine("\trm -rf ram*.h");
				makefile.WriteLine("\trm -rf *.vhd");
				makefile.WriteLine("\trm -rf *.elf");
				makefile.WriteLine();

                makefile.Write(all.ToString());
				//makefile.WriteLine("\n\t@read -p \"Press any key to continue...\"");
				makefile.WriteLine();

                makefile.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion

            progressBar.Value += 5;
            lblProgress.Text = "Generating simulation makefile";

            #region Simulation Makefile

            System.Text.StringBuilder SimMakefile = new System.Text.StringBuilder();
            StreamWriter SimMakefile_wr;

            /* Reads/Edits the file Makefile */
            try
            {
                SimMakefile.AppendLine("LIB=work");
                SimMakefile.AppendLine("");
				SimMakefile.AppendLine("#this environment variable must point to the hemps path, where the hardware, software and tools folders are located");
                //SimMakefile.AppendLine("HEMPS_PATH=" + Environment.CurrentDirectory);
                SimMakefile.AppendLine("BASE_PATH=$(HEMPS_PATH)");
                SimMakefile.AppendLine("HW_PATH=hardware");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("#VHDL files");
                SimMakefile.AppendLine("PKG_SRC=HeMPS_defaults.vhd");
                SimMakefile.AppendLine("PKG_DIR=$(HW_PATH)");
                SimMakefile.AppendLine("PKG_PATH=$(addprefix $(PKG_DIR)/,$(PKG_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("SCENARIO_SRC=HeMPS_PKG repository dynamic_apps");
                SimMakefile.AppendLine("SCENARIO_DIR=./");
                SimMakefile.AppendLine("SCENARIO_PATH=$(addprefix $(SCENARIO_DIR)/,$(SCENARIO_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("MPACK_SRC=mlite_pack.vhd UartFile.vhd");
                SimMakefile.AppendLine("MPACK_DIR=$(HW_PATH)/plasma/rtl");
                SimMakefile.AppendLine("MPACK_PATH=$(addprefix $(MPACK_DIR)/,$(MPACK_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("MLITE_SRC=alu.vhd bus_mux.vhd control.vhd mem_ctrl.vhd mult.vhd pc_next.vhd pipeline.vhd reg_bank.vhd shifter.vhd mlite_cpu.vhd");
                SimMakefile.AppendLine("MLITE_DIR=$(HW_PATH)/plasma/rtl");
                SimMakefile.AppendLine("MLITE_PATH=$(addprefix $(MLITE_DIR)/,$(MLITE_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("NI_SRC=network_interface.vhd");
                SimMakefile.AppendLine("NI_DIR=$(HW_PATH)/ni/rtl");
                SimMakefile.AppendLine("NI_PATH=$(NI_DIR)/$(NI_SRC)");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("DMA_SRC=dma.vhd dma_master.vhd");
                SimMakefile.AppendLine("DMA_DIR=$(HW_PATH)/dma/rtl");
                SimMakefile.AppendLine("DMA_PATH=$(addprefix $(DMA_DIR)/,$(DMA_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("ROUTER_SRC=Hermes_buffer.vhd Hermes_crossbar.vhd Hermes_switchcontrol.vhd RouterCC.vhd");
                SimMakefile.AppendLine("ROUTER_DIR=$(HW_PATH)/router/rtl");
                SimMakefile.AppendLine("ROUTER_PATH=$(addprefix $(ROUTER_DIR)/,$(ROUTER_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("PLASMA_RAM_SRC=ram_master ram_plasma");
                SimMakefile.AppendLine("PLASMA_RAM_DIR=$(SCENARIO_DIR)/plasma_ram/rtl");
                SimMakefile.AppendLine("PLASMA_RAM_PATH=$(addprefix $(PLASMA_RAM_DIR)/,$(PLASMA_RAM_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("MBLITE_RAM_SRC=ram_mblite");
                SimMakefile.AppendLine("MBLITE_RAM_DIR=$(SCENARIO_DIR)/mblite_ram/rtl");
                SimMakefile.AppendLine("MBLITE_RAM_PATH=$(MBLITE_RAM_DIR)/$(MBLITE_RAM_SRC)");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("MBLITE_SRC=mblite_soc.vhd");
                SimMakefile.AppendLine("MBLITE_DIR=$(HW_PATH)/mblite/rtl");
                SimMakefile.AppendLine("MBLITE_PATH=$(addprefix $(MBLITE_DIR)/,$(MBLITE_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("MBLITE_CORE_SRC=../config_Pkg.vhd core_Pkg.vhd core.vhd fetch.vhd gprf.vhd decode.vhd execute.vhd mem.vhd");
                SimMakefile.AppendLine("MBLITE_CORE_DIR=$(MBLITE_DIR)/core");
                SimMakefile.AppendLine("MBLITE_CORE_PATH=$(addprefix $(MBLITE_CORE_DIR)/,$(MBLITE_CORE_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("MBLITE_STD_SRC=std_Pkg.vhd dsram.vhd");
                SimMakefile.AppendLine("MBLITE_STD_DIR=$(MBLITE_DIR)/std");
                SimMakefile.AppendLine("MBLITE_STD_PATH=$(addprefix $(MBLITE_STD_DIR)/,$(MBLITE_STD_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("PLASMA_SRC=access_repository.vhd plasma.vhd");
                SimMakefile.AppendLine("PLASMA_DIR=$(HW_PATH)/plasma/rtl");
                SimMakefile.AppendLine("PLASMA_PATH=$(addprefix $(PLASMA_DIR)/,$(PLASMA_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("TOP_SRC=processing_element.vhd HeMPS.vhd insert_application.vhd test_bench.vhd");
                SimMakefile.AppendLine("TOP_DIR=$(HW_PATH)");
                SimMakefile.AppendLine("TOP_PATH=$(addprefix $(TOP_DIR)/,$(TOP_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("#SystemC files");
                SimMakefile.AppendLine("SC_NOC_DIR=$(HW_PATH)/router/sc");
                SimMakefile.AppendLine("SC_MLITE_DIR=$(HW_PATH)/plasma/sc");
                SimMakefile.AppendLine("SC_RAM_DIR=$(SCENARIO_DIR)/plasma_ram/sc");
                SimMakefile.AppendLine("SC_MBLITE_RAM_DIR=$(SCENARIO_DIR)/mblite_ram/sc");
                SimMakefile.AppendLine("SC_PLASMA_DIR=$(HW_PATH)/plasma/sc");
                SimMakefile.AppendLine("SC_TESTBENCH_DIR=$(HW_PATH)/sc");
                SimMakefile.AppendLine("SC_NI_DIR=$(HW_PATH)/ni/sc");
                SimMakefile.AppendLine("SC_DMA_DIR=$(HW_PATH)/dma/sc");
                SimMakefile.AppendLine("SC_ACCESS_REPO_DIR=$(HW_PATH)/plasma/sc");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("SC_NOC_SRC=queue.cpp switchcontrol.cpp router_cc.cpp ");
                SimMakefile.AppendLine("SC_MLITE_SRC=mlite_cpu.cpp");
                SimMakefile.AppendLine("SC_RAM_SRC=ram_master ram_plasma");
                SimMakefile.AppendLine("SC_MBLITE_RAM_SRC=ram_mblite");
                SimMakefile.AppendLine("SC_PLASMA_SRC=plasma_master.cpp plasma_slave.cpp");
                SimMakefile.AppendLine("SC_TESTBENCH_SRC=hemps.cpp test_bench.cpp");
                SimMakefile.AppendLine("SC_NI_SRC=Network_Interface.cpp");
                SimMakefile.AppendLine("SC_DMA_SRC=dma.cpp dma_master.cpp");
                SimMakefile.AppendLine("SC_ACCESS_REPO_SRC=access_repository.cpp");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("SC_NOC_PATH=$(addprefix $(SC_NOC_DIR)/,$(SC_NOC_SRC))");
                SimMakefile.AppendLine("SC_MLITE_PATH=$(addprefix $(SC_MLITE_DIR)/,$(SC_MLITE_SRC))");
                SimMakefile.AppendLine("SC_RAM_PATH=$(addprefix $(SC_RAM_DIR)/,$(SC_RAM_SRC))");
                SimMakefile.AppendLine("SC_MBLITE_RAM_PATH=$(SC_MBLITE_RAM_DIR)/$(SC_MBLITE_RAM_SRC)");
                SimMakefile.AppendLine("SC_PLASMA_PATH=$(addprefix $(SC_PLASMA_DIR)/,$(SC_PLASMA_SRC))");
                SimMakefile.AppendLine("SC_TESTBENCH_PATH=$(addprefix $(SC_TESTBENCH_DIR)/,$(SC_TESTBENCH_SRC))");
                SimMakefile.AppendLine("SC_NI_PATH=$(addprefix $(SC_NI_DIR)/,$(SC_NI_SRC))");
                SimMakefile.AppendLine("SC_DMA_PATH=$(addprefix $(SC_DMA_DIR)/,$(SC_DMA_SRC))");
                SimMakefile.AppendLine("SC_ACCESS_REPO_PATH=$(addprefix $(SC_ACCESS_REPO_DIR)/,$(SC_ACCESS_REPO_SRC))");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("#compilers definitions");
                SimMakefile.AppendLine("INC=-Iplasma_ram/sc/ -I./");
                SimMakefile.AppendLine("VHD_C=@vcom");
                SimMakefile.AppendLine("GEN=g++ -o HeMPS.exe *.o -L. -Linc/. -lsystemc");
                SimMakefile.AppendLine("");

                if (rbtnRtl.Checked || rbtnIss.Checked || rbtnScModelsim.Checked)
                {
                    SimMakefile.AppendLine("#modelsim gcc compiler");
                    SimMakefile.AppendLine("SC_C=@sccom -work $(LIB) -g");
                    SimMakefile.AppendLine("#systemc g++ compiler");
                    SimMakefile.AppendLine("#SC_C=g++ -c -g -Wall -O2");
                }
                else
                {
                    SimMakefile.AppendLine("#modelsim gcc compiler");
                    SimMakefile.AppendLine("#SC_C=@sccom -work $(LIB) -g");
                    SimMakefile.AppendLine("#systemc g++ compiler");
                    SimMakefile.AppendLine("SC_C=g++ -c -g -Wall -O2");
                }
				
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("default:");
                SimMakefile.AppendLine("	@echo \"Makefile for the hemps in systemc\"");
                SimMakefile.AppendLine("	@echo \"---------------------------------------\"");
                SimMakefile.AppendLine("	@echo \"Make options:\"");
                SimMakefile.AppendLine("	@echo \"   default: Echo these instructions\"");
                SimMakefile.AppendLine("	@echo \"       lib: Generate work dir and map its library\"");
                SimMakefile.AppendLine("	@echo \"       vhd: Compile vhd HeMPS description files\"");
                SimMakefile.AppendLine("	@echo \"       iss: Compile vhd and SystemC HeMPS description files\"");
                SimMakefile.AppendLine("	@echo \"       all: Compile sytemc and vhd files\"");
                SimMakefile.AppendLine("	@echo \"     clean: Remove all compiled and generated files\"");
                SimMakefile.AppendLine("	@echo");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("iss: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(SC_MLITE_PATH) $(NI_PATH) $(DMA_PATH) $(ROUTER_PATH) $(SC_RAM_PATH) $(MBLITE_STD_PATH) $(MBLITE_CORE_PATH) $(SC_MBLITE_RAM_PATH) $(MBLITE_PATH) $(PLASMA_PATH) $(TOP_PATH)");
                SimMakefile.AppendLine("	@sccom -link");
                SimMakefile.AppendLine("	");
				SimMakefile.AppendLine("");
				SimMakefile.AppendLine("scmod: lib $(SC_NOC_PATH) $(SC_MLITE_PATH) $(SC_RAM_PATH) $(SC_ACCESS_REPO_PATH) $(SC_DMA_PATH) $(SC_NI_PATH) $(SC_PLASMA_PATH) $(SC_TESTBENCH_PATH)");
				SimMakefile.AppendLine("	@sccom -link");
				SimMakefile.AppendLine("	");
				SimMakefile.AppendLine("");
                SimMakefile.AppendLine("sc: $(SC_NOC_PATH) $(SC_MLITE_PATH) $(SC_RAM_PATH) $(SC_ACCESS_REPO_PATH) $(SC_DMA_PATH) $(SC_NI_PATH) $(SC_PLASMA_PATH) $(SC_TESTBENCH_PATH)");
                SimMakefile.AppendLine("	$(GEN)");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("vhd: lib $(SCENARIO_PATH) $(PKG_PATH) $(MPACK_PATH) $(MLITE_PATH) $(NI_PATH) $(DMA_PATH) $(ROUTER_PATH) $(PLASMA_RAM_PATH) $(MBLITE_STD_PATH) $(MBLITE_CORE_PATH) $(MBLITE_RAM_PATH) $(MBLITE_PATH) $(PLASMA_PATH) $(TOP_PATH)");
                SimMakefile.AppendLine("\t");
                SimMakefile.AppendLine("$(SCENARIO_PATH):");
                SimMakefile.AppendLine("	$(VHD_C) -work $(LIB) $(@).vhd");
                SimMakefile.AppendLine("\t");
                SimMakefile.AppendLine("$(PLASMA_RAM_PATH):");
                SimMakefile.AppendLine("	$(VHD_C) -work $(LIB) $(@).vhd");
                SimMakefile.AppendLine("\t");
                SimMakefile.AppendLine("$(MBLITE_RAM_PATH):");
                SimMakefile.AppendLine("	$(VHD_C) -work $(LIB) $(@).vhd");
                SimMakefile.AppendLine("\t");
                SimMakefile.AppendLine("$(SC_RAM_PATH):");
                SimMakefile.AppendLine("	$(SC_C) $(@).cpp  $(INC)");
                SimMakefile.AppendLine("\t");
                SimMakefile.AppendLine("$(SC_MBLITE_RAM_PATH):");
                SimMakefile.AppendLine("\t$(SC_C) $(@).cpp  $(INC)");
                SimMakefile.AppendLine("");
                SimMakefile.AppendLine("%.vhd:");
                SimMakefile.AppendLine("	$(VHD_C) -work $(LIB) $(BASE_PATH)/$*.vhd");
                SimMakefile.AppendLine("	");
                SimMakefile.AppendLine("%.cpp:");
                SimMakefile.AppendLine("	$(SC_C) $(BASE_PATH)/$*.cpp  $(INC)");
                SimMakefile.AppendLine("	");
                SimMakefile.AppendLine("sim:");
                SimMakefile.AppendLine("	do sim.do");
                SimMakefile.AppendLine("	");
                SimMakefile.AppendLine("lib:");
                SimMakefile.AppendLine("	@vlib $(LIB)");
                SimMakefile.AppendLine("	@vmap $(LIB) $(LIB)");
                SimMakefile.AppendLine("	");
                SimMakefile.AppendLine("clean:");
                SimMakefile.AppendLine("	@rm -r -f $(LIB)");
                SimMakefile.AppendLine("	@rm -f transcript");
                SimMakefile.AppendLine("	@rm -f modelsim.ini");
                SimMakefile.AppendLine("	@rm -f vsim.wlf");
                SimMakefile.AppendLine("	@rm -f *~");
                SimMakefile.AppendLine("	@rm -f *.o");
                SimMakefile.AppendLine("	@rm -f *.exe");
                SimMakefile.AppendLine("	");
                if (rbtnRtl.Checked)
                {
                    SimMakefile.AppendLine("all: vhd");
                }
                else if (rbtnSc.Checked)
                {
                    SimMakefile.AppendLine("all: sc");
                }
                else if (rbtnIss.Checked)
                {
                    SimMakefile.AppendLine("all: iss");
                }
				else if (rbtnScModelsim.Checked)
				{
					SimMakefile.AppendLine("all: scmod");
				}
				
                SimMakefile.AppendLine("	");

                /* Writes the new Makefile file */
                SimMakefile_wr = new StreamWriter(projectPath + "//Makefile");
                SimMakefile_wr.Write(SimMakefile.ToString());
                SimMakefile_wr.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion

            progressBar.Value += 5;
            lblProgress.Text = "Generating sim.do";

            #region sim.do Generation
            /*System.Text.StringBuilder sim_do = new System.Text.StringBuilder();
            StreamWriter sim_do_wr;

            // Reads/Edits the file sim.do
            /*try
            {
                //sim_do.AppendLine("make all");
                sim_do.AppendLine("vsim -t ps +notimingchecks work.test_bench"); //-Gmlite_description=ISS -Gram_description=ISS 
                sim_do.AppendLine("");
                sim_do.AppendLine("do wave.do");
                sim_do.AppendLine("onerror {resume}");
                sim_do.AppendLine("radix hex");
                sim_do.AppendLine("set NumericStdNoWarnings 1");
                sim_do.AppendLine("set StdArithNoWarnings 1");
                sim_do.AppendLine("");
                sim_do.AppendLine("run " + txtbSimDuration.Text + " " + dudTime.Text);

                // Writes the new Makefile file
                sim_do_wr = new StreamWriter(projectPath + "//sim.do");
                sim_do_wr.Write(sim_do.ToString());
                sim_do_wr.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }*/
            #endregion

            progressBar.Value += 5;
            lblProgress.Text = "Generating wave.do";

            #region wave.do Generation
            System.Text.StringBuilder wave_do = new System.Text.StringBuilder();
            StreamWriter wave_do_wr;

            /* Reads/Edits the file sim.do*/
            try
            {
				if(rbtnScModelsim.Checked){
					
	                wave_do.AppendLine("onerror {resume}");
	                wave_do.AppendLine("quietly WaveActivateNextPane {} 0");
	                wave_do.AppendLine("add wave -noupdate -divider HeMPS");
	                wave_do.AppendLine("add wave -noupdate -divider {Master XX - " + masterAddress + "}");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/master"+ masterAddress +"/router/tx(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/master"+ masterAddress +"/router/data_out(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/master"+ masterAddress +"/router/credit_i(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/master"+ masterAddress +"/router/credit_o(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/master"+ masterAddress +"/router/rx(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/master"+ masterAddress +"/router/data_in(4)");
	                wave_do.AppendLine("add wave -noupdate -divider repository");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/read_req");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/mem_addr");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/data_read");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/data_valid");
	                wave_do.AppendLine("add wave -noupdate -divider {debug messages}");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/write_enable_debug");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/data_out_debug");
	                wave_do.AppendLine("add wave -noupdate -radix ascii /test_bench/hemps/data_out_debug");
	                wave_do.AppendLine("add wave -noupdate -divider Slaves");
	
	                for (int i = 0; i <= slaveProcessors; i++)
	                {
	                    if (i != masterAddress)
	                    {
	                        wave_do.AppendLine("add wave -noupdate -divider {slave 00 - " + i + "}");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/slave"+ i +"/router/address");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/slave"+ i +"/router/tx(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/slave"+ i +"/router/data_out(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/slave"+ i +"/router/credit_i(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/slave"+ i +"/router/credit_o(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/slave"+ i +"/router/rx(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/hemps/slave"+ i +"/router/data_in(4)");
	                    };
	                }
	
	                wave_do.AppendLine("TreeUpdate [SetDefaultTree]");
	                wave_do.AppendLine("WaveRestoreCursors {{Cursor 1} {1627970253 ps} 0}");
	                wave_do.AppendLine("configure wave -namecolwidth 190");
	                wave_do.AppendLine("configure wave -valuecolwidth 100");
	                wave_do.AppendLine("configure wave -justifyvalue left");
	                wave_do.AppendLine("configure wave -signalnamewidth 1");
	                wave_do.AppendLine("configure wave -snapdistance 10");
	                wave_do.AppendLine("configure wave -datasetprefix 0");
	                wave_do.AppendLine("configure wave -rowmargin 4");
	                wave_do.AppendLine("configure wave -childrowmargin 2");
	                wave_do.AppendLine("configure wave -gridoffset 0");
	                wave_do.AppendLine("configure wave -gridperiod 1");
	                wave_do.AppendLine("configure wave -griddelta 40");
	                wave_do.AppendLine("configure wave -timeline 0");
	                wave_do.AppendLine("configure wave -timelineunits ps");
	                wave_do.AppendLine("update");
	                wave_do.AppendLine("WaveRestoreZoom {0 ps} {3198211064 ps}");
	
	                /* Writes the new Makefile file */
	                wave_do_wr = new StreamWriter(projectPath + "//wave.do");
	                wave_do_wr.Write(wave_do.ToString());
	                wave_do_wr.Close();
					
				}
				
			else				//VHDL or ISS
				{
					wave_do.AppendLine("onerror {resume}");
	                wave_do.AppendLine("quietly WaveActivateNextPane {} 0");
	                wave_do.AppendLine("add wave -noupdate -divider HeMPS");
	                wave_do.AppendLine("add wave -noupdate -divider {Master XX - " + masterAddress + "}");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + masterAddress + ")/mas/master/ROUTER_RTL/routerCC/tx(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + masterAddress + ")/mas/master/ROUTER_RTL/routerCC/data_out(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + masterAddress + ")/mas/master/ROUTER_RTL/routerCC/credit_i(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + masterAddress + ")/mas/master/ROUTER_RTL/routerCC/credit_o(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + masterAddress + ")/mas/master/ROUTER_RTL/routerCC/rx(4)");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + masterAddress + ")/mas/master/ROUTER_RTL/routerCC/data_in(4)");
	                wave_do.AppendLine("add wave -noupdate -divider repository");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/read_req");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/mem_addr");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/data_read");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/data_valid");
	                wave_do.AppendLine("add wave -noupdate -divider {debug messages}");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/write_enable_debug");
	                wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/data_out_debug");
	                wave_do.AppendLine("add wave -noupdate -radix ascii /test_bench/HeMPS/data_out_debug");
	                wave_do.AppendLine("add wave -noupdate -divider Slaves");
	
	                for (int i = 0; i <= slaveProcessors; i++)
	                {
	                    if (i != masterAddress)
	                    {
	                        wave_do.AppendLine("add wave -noupdate -divider {slave 00 - " + i + "}");
	                        wave_do.AppendLine("add wave -noupdate /test_bench/HeMPS/proc(" + i + ")/slav/slave/core_type");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + i + ")/slav/slave/ROUTER_RTL/routerCC/address");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + i + ")/slav/slave/ROUTER_RTL/routerCC/tx(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + i + ")/slav/slave/ROUTER_RTL/routerCC/data_out(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + i + ")/slav/slave/ROUTER_RTL/routerCC/credit_i(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + i + ")/slav/slave/ROUTER_RTL/routerCC/credit_o(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + i + ")/slav/slave/ROUTER_RTL/routerCC/rx(4)");
	                        wave_do.AppendLine("add wave -noupdate -radix hexadecimal /test_bench/HeMPS/proc(" + i + ")/slav/slave/ROUTER_RTL/routerCC/data_in(4)");
	                    };
	                }
	
	                wave_do.AppendLine("TreeUpdate [SetDefaultTree]");
	                wave_do.AppendLine("WaveRestoreCursors {{Cursor 1} {1627970253 ps} 0}");
	                wave_do.AppendLine("configure wave -namecolwidth 190");
	                wave_do.AppendLine("configure wave -valuecolwidth 100");
	                wave_do.AppendLine("configure wave -justifyvalue left");
	                wave_do.AppendLine("configure wave -signalnamewidth 1");
	                wave_do.AppendLine("configure wave -snapdistance 10");
	                wave_do.AppendLine("configure wave -datasetprefix 0");
	                wave_do.AppendLine("configure wave -rowmargin 4");
	                wave_do.AppendLine("configure wave -childrowmargin 2");
	                wave_do.AppendLine("configure wave -gridoffset 0");
	                wave_do.AppendLine("configure wave -gridperiod 1");
	                wave_do.AppendLine("configure wave -griddelta 40");
	                wave_do.AppendLine("configure wave -timeline 0");
	                wave_do.AppendLine("configure wave -timelineunits ps");
	                wave_do.AppendLine("update");
	                wave_do.AppendLine("WaveRestoreZoom {0 ps} {3198211064 ps}");
	
	                /* Writes the new Makefile file */
	                wave_do_wr = new StreamWriter(projectPath + "//wave.do");
	                wave_do_wr.Write(wave_do.ToString());
	                wave_do_wr.Close();
					
					
					
					
					
				}
            }
				
				
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion

            //progressBar.Value += 10;
            lblProgress.Text = "Generating ram_master.cpp and ram_slave.cpp";

            #region ram_master.cpp and ram_plasma.cpp and ram_mblite.cpp

            System.Text.StringBuilder ram_plasma_cpp = new System.Text.StringBuilder();
            StreamWriter ram_plasma_cpp_wr;
            System.Text.StringBuilder ram_mblite_cpp = new System.Text.StringBuilder();
            StreamWriter ram_mblite_cpp_wr;
            System.Text.StringBuilder ram_master_cpp = new System.Text.StringBuilder();
            StreamWriter ram_master_cpp_wr;

            try
            {
                ram_plasma_cpp.AppendLine("#include \"ram_plasma.h\"");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("#ifdef MTI_SYSTEMC");
                ram_plasma_cpp.AppendLine("SC_MODULE_EXPORT(ram_plasma);");
                ram_plasma_cpp.AppendLine("#endif");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("/*** Memory read port A ***/");
                ram_plasma_cpp.AppendLine("void ram_plasma::read_a() {");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	unsigned int address;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	address = (unsigned int)address_a.read();");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	if ( address < RAM_SIZE )");
                ram_plasma_cpp.AppendLine("		data_read_a.write(ram[address]);");
                ram_plasma_cpp.AppendLine("}");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("/*** Memory write port A ***/");
                ram_plasma_cpp.AppendLine("void ram_plasma::write_a() {");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	unsigned int data, address;");
                ram_plasma_cpp.AppendLine("	unsigned char wbe;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	wbe = (unsigned char)wbe_a.read();");
                ram_plasma_cpp.AppendLine("	address = (unsigned int)address_a.read();");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	if ( wbe != 0 && address < RAM_SIZE) {");
                ram_plasma_cpp.AppendLine("		data = ram[address];");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("		switch(wbe) {");
                ram_plasma_cpp.AppendLine("			case 0xF:	// Write word");
                ram_plasma_cpp.AppendLine("				ram[address] = data_write_a.read();");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 0xC:	// Write MSW");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~half_word[1]) | (data_write_a.read() & half_word[1]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 3:		// Write LSW");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~half_word[0]) | (data_write_a.read() & half_word[0]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 8:		// Write byte 3");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~byte[3]) | (data_write_a.read() & byte[3]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 4:		// Write byte 2");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~byte[2]) | (data_write_a.read() & byte[2]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 2:		// Write byte 1");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~byte[1]) | (data_write_a.read() & byte[1]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 1:		// Write byte 0");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~byte[0]) | (data_write_a.read() & byte[0]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("		}");
                ram_plasma_cpp.AppendLine("	}");
                ram_plasma_cpp.AppendLine("}");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("/*** Memory read port B ***/");
                ram_plasma_cpp.AppendLine("void ram_plasma::read_b() {");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	unsigned int address;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	address = (unsigned int)address_b.read();");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	if ( address < RAM_SIZE )");
                ram_plasma_cpp.AppendLine("		data_read_b.write(ram[address]);");
                ram_plasma_cpp.AppendLine("}");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("/*** Memory write port B ***/");
                ram_plasma_cpp.AppendLine("void ram_plasma::write_b() {");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	unsigned int data, address;");
                ram_plasma_cpp.AppendLine("	unsigned char wbe;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	wbe = (unsigned char)wbe_b.read();");
                ram_plasma_cpp.AppendLine("	address = (unsigned int)address_b.read();");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("	if ( wbe != 0 && address < RAM_SIZE) {");
                ram_plasma_cpp.AppendLine("		data = ram[address];");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("		switch(wbe) {");
                ram_plasma_cpp.AppendLine("			case 0xF:	// Write word");
                ram_plasma_cpp.AppendLine("				ram[address] = data_write_b.read();");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 0xC:	// Write MSW");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~half_word[1]) | (data_write_b.read() & half_word[1]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 3:		// Write LSW");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~half_word[0]) | (data_write_b.read() & half_word[0]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 8:		// Write byte 3");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~byte[3]) | (data_write_b.read() & byte[3]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 4:		// Write byte 2");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~byte[2]) | (data_write_b.read() & byte[2]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 2:		// Write byte 1");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~byte[1]) | (data_write_b.read() & byte[1]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("");
                ram_plasma_cpp.AppendLine("			case 1:		// Write byte 0");
                ram_plasma_cpp.AppendLine("				ram[address] = (data & ~byte[0]) | (data_write_b.read() & byte[0]);");
                ram_plasma_cpp.AppendLine("			break;");
                ram_plasma_cpp.AppendLine("		}");
                ram_plasma_cpp.AppendLine("	}");
                ram_plasma_cpp.AppendLine("}");
                ram_plasma_cpp.AppendLine("");

                /* Writes the new Makefile file */
                ram_plasma_cpp_wr = new StreamWriter(projectPath + "//plasma_ram//sc//ram_plasma.cpp");
                ram_plasma_cpp_wr.Write(ram_plasma_cpp.ToString());
                ram_plasma_cpp_wr.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }

            try
            {
                ram_mblite_cpp.AppendLine("#include \"ram_mblite.h\"");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("#ifdef MTI_SYSTEMC");
                ram_mblite_cpp.AppendLine("SC_MODULE_EXPORT(ram_mblite);");
                ram_mblite_cpp.AppendLine("#endif");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("/*** Memory read port A ***/");
                ram_mblite_cpp.AppendLine("void ram_mblite::read_a() {");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	unsigned int address;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	address = (unsigned int)address_a.read();");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	if ( address < RAM_SIZE )");
                ram_mblite_cpp.AppendLine("		data_read_a.write(ram[address]);");
                ram_mblite_cpp.AppendLine("}");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("/*** Memory write port A ***/");
                ram_mblite_cpp.AppendLine("void ram_mblite::write_a() {");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	unsigned int data, address;");
                ram_mblite_cpp.AppendLine("	unsigned char wbe;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	wbe = (unsigned char)wbe_a.read();");
                ram_mblite_cpp.AppendLine("	address = (unsigned int)address_a.read();");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	if ( wbe != 0 && address < RAM_SIZE) {");
                ram_mblite_cpp.AppendLine("		data = ram[address];");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("		switch(wbe) {");
                ram_mblite_cpp.AppendLine("			case 0xF:	// Write word");
                ram_mblite_cpp.AppendLine("				ram[address] = data_write_a.read();");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 0xC:	// Write MSW");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~half_word[1]) | (data_write_a.read() & half_word[1]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 3:		// Write LSW");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~half_word[0]) | (data_write_a.read() & half_word[0]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 8:		// Write byte 3");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~byte[3]) | (data_write_a.read() & byte[3]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 4:		// Write byte 2");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~byte[2]) | (data_write_a.read() & byte[2]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 2:		// Write byte 1");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~byte[1]) | (data_write_a.read() & byte[1]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 1:		// Write byte 0");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~byte[0]) | (data_write_a.read() & byte[0]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("		}");
                ram_mblite_cpp.AppendLine("	}");
                ram_mblite_cpp.AppendLine("}");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("/*** Memory read port B ***/");
                ram_mblite_cpp.AppendLine("void ram_mblite::read_b() {");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	unsigned int address;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	address = (unsigned int)address_b.read();");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	if ( address < RAM_SIZE )");
                ram_mblite_cpp.AppendLine("		data_read_b.write(ram[address]);");
                ram_mblite_cpp.AppendLine("}");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("/*** Memory write port B ***/");
                ram_mblite_cpp.AppendLine("void ram_mblite::write_b() {");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	unsigned int data, address;");
                ram_mblite_cpp.AppendLine("	unsigned char wbe;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	wbe = (unsigned char)wbe_b.read();");
                ram_mblite_cpp.AppendLine("	address = (unsigned int)address_b.read();");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("	if ( wbe != 0 && address < RAM_SIZE) {");
                ram_mblite_cpp.AppendLine("		data = ram[address];");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("		switch(wbe) {");
                ram_mblite_cpp.AppendLine("			case 0xF:	// Write word");
                ram_mblite_cpp.AppendLine("				ram[address] = data_write_b.read();");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 0xC:	// Write MSW");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~half_word[1]) | (data_write_b.read() & half_word[1]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 3:		// Write LSW");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~half_word[0]) | (data_write_b.read() & half_word[0]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 8:		// Write byte 3");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~byte[3]) | (data_write_b.read() & byte[3]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 4:		// Write byte 2");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~byte[2]) | (data_write_b.read() & byte[2]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 2:		// Write byte 1");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~byte[1]) | (data_write_b.read() & byte[1]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("");
                ram_mblite_cpp.AppendLine("			case 1:		// Write byte 0");
                ram_mblite_cpp.AppendLine("				ram[address] = (data & ~byte[0]) | (data_write_b.read() & byte[0]);");
                ram_mblite_cpp.AppendLine("			break;");
                ram_mblite_cpp.AppendLine("		}");
                ram_mblite_cpp.AppendLine("	}");
                ram_mblite_cpp.AppendLine("}");
                ram_mblite_cpp.AppendLine("");

                /* Writes the new Makefile file */
                ram_mblite_cpp_wr = new StreamWriter(projectPath + "/mblite_ram/sc/ram_mblite.cpp");
                ram_mblite_cpp_wr.Write(ram_mblite_cpp.ToString());
                ram_mblite_cpp_wr.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }


            try
            {
                ram_master_cpp.AppendLine("#include \"ram_master.h\"");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("#ifdef MTI_SYSTEMC");
                ram_master_cpp.AppendLine("SC_MODULE_EXPORT(ram_master);");
                ram_master_cpp.AppendLine("#endif");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("/*** Memory read port A ***/");
                ram_master_cpp.AppendLine("void ram_master::read_a() {");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	unsigned int address;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	address = (unsigned int)address_a.read();");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	if ( address < RAM_SIZE )");
                ram_master_cpp.AppendLine("		data_read_a.write(ram[address]);");
                ram_master_cpp.AppendLine("}");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("/*** Memory write port A ***/");
                ram_master_cpp.AppendLine("void ram_master::write_a() {");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	unsigned int data, address;");
                ram_master_cpp.AppendLine("	unsigned char wbe;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	wbe = (unsigned char)wbe_a.read();");
                ram_master_cpp.AppendLine("	address = (unsigned int)address_a.read();");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	if ( wbe != 0 && address < RAM_SIZE) {");
                ram_master_cpp.AppendLine("		data = ram[address];");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("		switch(wbe) {");
                ram_master_cpp.AppendLine("			case 0xF:	// Write word");
                ram_master_cpp.AppendLine("				ram[address] = data_write_a.read();");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 0xC:	// Write MSW");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~half_word[1]) | (data_write_a.read() & half_word[1]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 3:		// Write LSW");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~half_word[0]) | (data_write_a.read() & half_word[0]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 8:		// Write byte 3");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~byte[3]) | (data_write_a.read() & byte[3]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 4:		// Write byte 2");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~byte[2]) | (data_write_a.read() & byte[2]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 2:		// Write byte 1");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~byte[1]) | (data_write_a.read() & byte[1]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 1:		// Write byte 0");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~byte[0]) | (data_write_a.read() & byte[0]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("		}");
                ram_master_cpp.AppendLine("	}");
                ram_master_cpp.AppendLine("}");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("/*** Memory read port B ***/");
                ram_master_cpp.AppendLine("void ram_master::read_b() {");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	unsigned int address;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	address = (unsigned int)address_b.read();");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	if ( address < RAM_SIZE )");
                ram_master_cpp.AppendLine("		data_read_b.write(ram[address]);");
                ram_master_cpp.AppendLine("}");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("/*** Memory write port B ***/");
                ram_master_cpp.AppendLine("void ram_master::write_b() {");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	unsigned int data, address;");
                ram_master_cpp.AppendLine("	unsigned char wbe;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	wbe = (unsigned char)wbe_b.read();");
                ram_master_cpp.AppendLine("	address = (unsigned int)address_b.read();");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("	if ( wbe != 0 && address < RAM_SIZE) {");
                ram_master_cpp.AppendLine("		data = ram[address];");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("		switch(wbe) {");
                ram_master_cpp.AppendLine("			case 0xF:	// Write word");
                ram_master_cpp.AppendLine("				ram[address] = data_write_b.read();");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 0xC:	// Write MSW");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~half_word[1]) | (data_write_b.read() & half_word[1]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 3:		// Write LSW");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~half_word[0]) | (data_write_b.read() & half_word[0]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 8:		// Write byte 3");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~byte[3]) | (data_write_b.read() & byte[3]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 4:		// Write byte 2");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~byte[2]) | (data_write_b.read() & byte[2]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 2:		// Write byte 1");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~byte[1]) | (data_write_b.read() & byte[1]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("");
                ram_master_cpp.AppendLine("			case 1:		// Write byte 0");
                ram_master_cpp.AppendLine("				ram[address] = (data & ~byte[0]) | (data_write_b.read() & byte[0]);");
                ram_master_cpp.AppendLine("			break;");
                ram_master_cpp.AppendLine("		}");
                ram_master_cpp.AppendLine("	}");
                ram_master_cpp.AppendLine("}");
                ram_master_cpp.AppendLine("");

                /* Writes the new Makefile file */
                ram_master_cpp_wr = new StreamWriter(projectPath + "//plasma_ram//sc//ram_master.cpp");
                ram_master_cpp_wr.Write(ram_master_cpp.ToString());
                ram_master_cpp_wr.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion

            progressBar.Value += 5;
            lblProgress.Text = "Generating application IDs";

            /* Generates the application IDs files */
            #region Application IDs

            /*  Gets all applications */
            foreach (TreeNode tn in tvApplications.Nodes)
            {

                /* Clean the StringBuilder */
                tasks.Remove(0, tasks.Length);

                /* Searches allocated task belonging to an applications */
                foreach (Task task in TaskIDs)
                    if (object.ReferenceEquals(task.Application, tn)) tasks.AppendLine("#define " + task.Text.Remove(task.Text.IndexOf('.')) + "\t" + TaskIDs.IndexOf(task));

                /* Verifies if there is some task allocated to the current application */
                if (tasks.Length > 0)
                {
                    try
                    {
                        idFile = new StreamWriter(projectPath + "/build/ids_" + tn.Text + ".h");
                        idFile.Write(tasks.ToString());
                        idFile.Close();
                    }
                    catch (System.Exception ex)
                    {
                        MessageBox.Show(ex.ToString());
                        return;
                    }
                }
            }
            #endregion

            progressBar.Value += 5;
            lblProgress.Text = "Generating Application Tasks Dependences";

            #region Application Tasks Dependences
            StreamReader taskFile;
            string straux, line2, task2, current_task, last_task;
            int flits;
            int dep_ok;
            taskList auxTask;
            dependence auxDependence;
            List<dependence> auxDpList = new List<dependence>();
            StringBuilder defApp = new StringBuilder();
            StringBuilder taskVec = new StringBuilder();


            try
            {
                /*  Gets all applications */
                foreach (TreeNode tn2 in tvApplications.Nodes)
                {
                    applicationDirectory = tn2.ToolTipText;
                    List<Task> InitTasks = new List<Task>();
                    /* Searches allocated task belonging to an applications */
                    foreach (Task lvi in TaskIDs)
                    {
                        if (object.ReferenceEquals(lvi.Application, tn2))
                        {
                            int receiveok = 0;
                            int sendok = 0;
                            dep_ok = 0;
                            task2 = lvi.Text.ToUpper();
							auxTask = new taskList(task2, tn2.Text);
							taskFile = new StreamReader(new FileStream(projectPath + "/applications/" + tn2.Text + "/" + lvi.Text, FileMode.Open, FileAccess.Read, FileShare.ReadWrite));
                            while ((line2 = taskFile.ReadLine()) != null)
                            {
                                if (line2.IndexOf("/*Comm ") >= 0)
                                {
                                    straux = line2.Substring(line2.IndexOf("/*Comm ", 0) + 7);
                                    straux = straux.Substring(0, straux.IndexOf("*/", 0));
                                    current_task = straux.Split(' ')[0] + ".C";
                                    flits = int.Parse(straux.Split(' ')[1]);
                                    auxDependence = auxTask.depList.Find(delegate(dependence dp) { return dp.task == current_task; });
                                    if (auxDependence == null)
                                    {
                                        auxDependence = new dependence(current_task, flits);
                                        auxTask.depList.Add(auxDependence);
                                    }
                                    else
                                    {
                                        auxDependence.flits = auxDependence.flits + flits;
                                    }
                                    dep_ok++;
                                }
                                if (line2.IndexOf("Receive") >= 0)
                                {
                                    receiveok = 1;
                                }

                                if (line2.IndexOf("Send") >= 0 && receiveok == 0 && sendok == 0)
                                {
                                    if (!InitTasks.Contains(lvi)) InitTasks.Add(lvi);
                                    sendok = 1;
                                }

                            }
                            taskFile.Close();

                            last_task = "";
                            if (dep_ok == 0)
                            {
								taskFile = new StreamReader(new FileStream(projectPath + "/applications/" + tn2.Text + "/" + lvi.Text, FileMode.Open, FileAccess.Read, FileShare.ReadWrite));
                                //MessageBox.Show(projectPath + "/applications/" + tn2.Text + "/" + lvi.Text);
                                while ((line2 = taskFile.ReadLine()) != null)
                                {
                                    if (line2.IndexOf("Send") >= 0 || line2.IndexOf("Receive") >= 0)
                                    {
                                        if (dep_ok == 0)
                                        {
                                            straux = line2.Substring(line2.IndexOf(",", 0) + 1);
                                            current_task = straux.Remove(straux.IndexOf(')'));
                                            current_task = current_task.ToUpper().Trim() + ".C";
                                            if (!String.Equals(last_task, current_task))
                                            {
                                                auxDependence = auxTask.depList.Find(delegate(dependence dp) { return dp.task == current_task; });
                                                if (auxDependence == null)
                                                {
                                                    auxDependence = new dependence(current_task, 100);
                                                    auxTask.depList.Add(auxDependence);
                                                }
                                                last_task = current_task;
                                            }
                                        }
                                    }
                                }
                                taskFile.Close();
                            }

                            auxTask.depList.Sort(delegate(dependence x, dependence y)
                            {
                                if (x.flits > y.flits) return -1;
                                if (x.flits < y.flits) return 1;
                                else return 0;
                            }
                                                );
                            tskList.Add(auxTask);
                        }
                    }
                    AppsInitTasks.Add(tn2.Text, InitTasks);
                }
                idFile = File.CreateText(projectPath + "//build//ApplicationsMatrix.h");
                foreach (taskList tsk in tskList)
                {
                    idFile.WriteLine("#TASK : " + tsk.task);
                    foreach (dependence dp in tsk.depList)
                    {
                        idFile.WriteLine("\t" + dp.task);
                    }
                }
                idFile.Close();

            }
            catch (System.Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion

            progressBar.Value += 5;
            lblProgress.Text = "Generating IDs master";

            /* Generates the ids_master.h file */
            #region ids_master

            //StringBuilder initializeProcessors = new StringBuilder();
            StringBuilder defines = new StringBuilder();

            //initializeProcessors.AppendLine("void InitializeProcessors() {");
			
			string[,] pe_type = new string[int.Parse(nudX.Value.ToString()), int.Parse(nudY.Value.ToString())];
			int plasma_processors = 0;
			int mblite_processors = 0;
   	        foreach (ProcessingElement pe in tlpProcessingElements.Controls)
            {
                    //string address = pe.X.ToString("X1") + pe.Y.ToString("X1");

                    //defines.AppendLine("#define SLAVE" + address + "\t" + "0x" + address);
                    if (pe.Plasma)
					{
						pe_type[pe.X, pe.Y] = "PLASMA";
						if(pe.Slave) plasma_processors++;
                        //initializeProcessors.AppendLine("\tInsertProc(SLAVE" + address + ",PLASMA);");
					}
                    else
					{
						pe_type[pe.X, pe.Y] = "MBLITE";
						mblite_processors++;
                        //initializeProcessors.AppendLine("\tInsertProc(SLAVE" + address + ",MBLITE);");
					}
            }

            //initializeProcessors.AppendLine("}");

            defines.AppendLine();
            defines.AppendLine("#define MAX_PROCESSORS\t\t" + slaveProcessors + "\t/* Number of slave processors available in the platform */");
            defines.AppendLine("#define MAX_LOCAL_TASKS\t\t" + tbxMaxTasksSlave.Text + "\t/* Number of task which can be allocated simultaneously in one processor */");
            defines.AppendLine("#define MAX_GLOBAL_TASKS\t" + "MAX_LOCAL_TASKS * MAX_PROCESSORS\t" + "/* Number of task which can be allocated simultaneously in the platform */");
            defines.AppendLine("#define PLASMA\t0");
            defines.AppendLine("#define MBLITE\t1");
			defines.AppendLine("#define PLASMA_PROCESSORS\t\t"+plasma_processors.ToString());
            defines.AppendLine("#define MBLITE_PROCESSORS\t\t"+mblite_processors.ToString());
            defines.AppendLine("#define XDIMENSION\t\t" + nudX.Value.ToString());
            defines.AppendLine("#define YDIMENSION\t\t" + nudY.Value.ToString());
            defines.AppendLine("#define MASTERADDRESS\t\t" + "0x" + masterAddress_ids_slave);
			defines.AppendLine("");
			defines.Append("char pe_free_pages[XDIMENSION][YDIMENSION] = {");
			for(int xd=1; xd<nudX.Value; xd++) 
			{
				defines.Append("{");
				for(int yd=1; yd<nudY.Value; yd++) 
				{	
					defines.Append("MAX_LOCAL_TASKS, ");
				}
				defines.Append("MAX_LOCAL_TASKS}, ");
			}
			defines.Append("{");
			for(int yd=1; yd<nudY.Value; yd++) 
			{	
				defines.Append("MAX_LOCAL_TASKS, ");
			}
			defines.Append("MAX_LOCAL_TASKS}};\n");
			defines.AppendLine("");
			defines.Append("char pe_type[XDIMENSION][YDIMENSION] = {");
			for(int xd=0; xd<nudX.Value; xd++) 
			{
				defines.Append("{");
				for(int yd=0; yd<nudY.Value; yd++) 
				{	
					defines.Append(pe_type[xd, yd]);
					if(yd==(nudY.Value-1)) defines.Append("}");
					else defines.Append(", ");
				}
				if(xd==(nudX.Value-1)) defines.Append("};");
				else defines.Append(", ");
			}
			defines.AppendLine("");
			defines.AppendLine("");
			defines.AppendLine("int total_free_pes = MAX_GLOBAL_TASKS;");
			defines.AppendLine("int free_plasmas = PLASMA_PROCESSORS * MAX_LOCAL_TASKS;");
			defines.AppendLine("int free_mblites = MBLITE_PROCESSORS * MAX_LOCAL_TASKS;");

            try
            {
                idFile = new StreamWriter(projectPath + "//build//ids_master.h");

                idFile.WriteLine("/* Slave processors addresses */");
                idFile.WriteLine(defines);

                idFile.WriteLine("void InsertTaskLoc(int id, int processor);");
                idFile.WriteLine("void PageUsed(int proc);\n");

                //idFile.WriteLine(initializeProcessors);
                idFile.Close();
            }
            catch (System.Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion

            progressBar.Value += 5;
            lblProgress.Text = "Generating IDs slave";

            /* Generates the ids_slave.h file */
            #region ids_slave

            try
            {
                idFile = new StreamWriter(projectPath + "/build/ids_slave.h");

                idFile.WriteLine("#define MAX_PROCESSORS\t\t" + slaveProcessors + "\t/* Number of slave processors available in the platform */");
                idFile.WriteLine("#define MAXLOCALTASKS\t\t" + tbxMaxTasksSlave.Text + "\t/* Number of task which can be allocated simultaneously in one processor */");
                idFile.WriteLine("#define MAX_GLOBAL_TASKS\t" + "MAXLOCALTASKS * MAX_PROCESSORS\t" + "/* Number of task which can be allocated simultaneously in the platform */");
                idFile.WriteLine("#define KERNELPAGECOUNT\t" + kernelPages);
                idFile.WriteLine("#define PAGESIZE\t\t\t" + Convert.ToUInt16(dudPageSize.Text) * 1024);
                idFile.WriteLine("#define MASTERADDRESS\t\t" + "0x" + masterAddress_ids_slave);
                idFile.WriteLine("#define XDIMENSION\t\t" + nudX.Value.ToString());
                idFile.WriteLine("#define YDIMENSION\t\t" + nudY.Value.ToString());

                idFile.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion
			
			/* Generates the InitializeVectorsSlave.h file */
            #region InitializeVectorsSlave

            StringBuilder InitializeVectorsSlave = new StringBuilder();
            
            InitializeVectorsSlave.AppendLine("/*--------------------------------------------------------------------");
			InitializeVectorsSlave.AppendLine(" * struct TaskLocation");
			InitializeVectorsSlave.AppendLine(" *");
			InitializeVectorsSlave.AppendLine(" * DESCRIPTION:");
			InitializeVectorsSlave.AppendLine(" *    Makes the task versus processor association");
			InitializeVectorsSlave.AppendLine(" *");
			InitializeVectorsSlave.AppendLine(" *--------------------------------------------------------------------*/");
			InitializeVectorsSlave.AppendLine("typedef struct {");
			InitializeVectorsSlave.AppendLine("\tint task;");
			InitializeVectorsSlave.AppendLine("\tint processor;");
			InitializeVectorsSlave.AppendLine("} TaskLocation;");
			InitializeVectorsSlave.AppendLine("");
			InitializeVectorsSlave.Append("TaskLocation task_location[MAX_GLOBAL_TASKS] = {");
			for(int v=1; v<(slaveProcessors*int.Parse(tbxMaxTasksSlave.Text)); v++) InitializeVectorsSlave.Append("{-1, -1}, ");
			InitializeVectorsSlave.Append("{-1, -1}};\n");
			InitializeVectorsSlave.AppendLine("");
			InitializeVectorsSlave.Append("int request_task[MAX_GLOBAL_TASKS] = {");
			for(int v=1; v<(slaveProcessors*int.Parse(tbxMaxTasksSlave.Text)); v++) InitializeVectorsSlave.Append("-1, ");
			InitializeVectorsSlave.Append("-1 };\n");
			InitializeVectorsSlave.AppendLine("");
			InitializeVectorsSlave.Append("int location_request_task[MAX_GLOBAL_TASKS] = {");
			for(int v=1; v<(slaveProcessors*int.Parse(tbxMaxTasksSlave.Text)); v++) InitializeVectorsSlave.Append("-1, ");
			InitializeVectorsSlave.Append("-1 };\n");
			
			
			try
            {
                idFile = new StreamWriter(projectPath + "//build//InitializeVectorsSlave.h");
				
				idFile.WriteLine(InitializeVectorsSlave);
                idFile.Close();
            }
            catch (System.Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion
			
			/* Generates the InitializeVectorsMaster.h file */
            #region InitializeVectorsMaster

            StringBuilder InitializeVectorsMaster = new StringBuilder();
            
            InitializeVectorsMaster.AppendLine("/*--------------------------------------------------------------------");
			InitializeVectorsMaster.AppendLine(" * struct TaskLocation");
			InitializeVectorsMaster.AppendLine(" *");
			InitializeVectorsMaster.AppendLine(" * DESCRIPTION:");
			InitializeVectorsMaster.AppendLine(" *    Makes the task versus processor association");
			InitializeVectorsMaster.AppendLine(" *");
			InitializeVectorsMaster.AppendLine(" *--------------------------------------------------------------------*/");
			InitializeVectorsMaster.AppendLine("typedef struct {");
			InitializeVectorsMaster.AppendLine("\tint task;");
			InitializeVectorsMaster.AppendLine("\tint processor;");
			InitializeVectorsMaster.AppendLine("} TaskLocation;");
			InitializeVectorsMaster.AppendLine("");
			InitializeVectorsMaster.AppendLine("/*--------------------------------------------------------------------");
			InitializeVectorsMaster.AppendLine(" * struct TaskRequest");
			InitializeVectorsMaster.AppendLine(" *");
			InitializeVectorsMaster.AppendLine(" * DESCRIPTION:");
			InitializeVectorsMaster.AppendLine(" *    Store the task requests");
			InitializeVectorsMaster.AppendLine(" *");
			InitializeVectorsMaster.AppendLine(" *--------------------------------------------------------------------*/");
			InitializeVectorsMaster.AppendLine("typedef struct {");
			InitializeVectorsMaster.AppendLine("\tint requested_task;");
			InitializeVectorsMaster.AppendLine("\tint requesting_task;");
			InitializeVectorsMaster.AppendLine("\tint source_processor;");
			InitializeVectorsMaster.AppendLine("} TaskRequest;");
			InitializeVectorsMaster.AppendLine("");		
			InitializeVectorsMaster.AppendLine("/*--------------------------------------------------------------------");
			InitializeVectorsMaster.AppendLine(" * struct LocationRequest");
			InitializeVectorsMaster.AppendLine(" *");
			InitializeVectorsMaster.AppendLine(" * DESCRIPTION:");
			InitializeVectorsMaster.AppendLine(" *    Store the task location requests");
			InitializeVectorsMaster.AppendLine(" *");
			InitializeVectorsMaster.AppendLine(" *--------------------------------------------------------------------*/");
			InitializeVectorsMaster.AppendLine("typedef struct {");
			InitializeVectorsMaster.AppendLine("\tint requested_task;");
			InitializeVectorsMaster.AppendLine("\tint requesting_task;");
			InitializeVectorsMaster.AppendLine("\tint source_processor;");
			InitializeVectorsMaster.AppendLine("} LocationRequest;");
			InitializeVectorsMaster.AppendLine("");	
			InitializeVectorsMaster.Append("TaskLocation task_location[MAX_GLOBAL_TASKS] = {");
			for(int v=1; v<(slaveProcessors*int.Parse(tbxMaxTasksSlave.Text)); v++) InitializeVectorsMaster.Append("{-1, -1}, ");
			InitializeVectorsMaster.Append("{-1, -1}};\n");
			InitializeVectorsMaster.AppendLine("");
			InitializeVectorsMaster.Append("int task_pes[MAX_GLOBAL_TASKS*2] = {");
			for(int v=1; v<(slaveProcessors*int.Parse(tbxMaxTasksSlave.Text)); v++) InitializeVectorsMaster.Append("-1, ");
			InitializeVectorsMaster.Append("-1};\n");
			InitializeVectorsMaster.AppendLine("");
			InitializeVectorsMaster.Append("TaskRequest task_request[MAX_GLOBAL_TASKS*2] = {");
			for(int v=1; v<(slaveProcessors*int.Parse(tbxMaxTasksSlave.Text)*2); v++) InitializeVectorsMaster.Append("{-1, -1, -1}, ");
			InitializeVectorsMaster.Append("{-1, -1, -1}};\n");
			InitializeVectorsMaster.AppendLine("");
			InitializeVectorsMaster.Append("LocationRequest location_request[MAX_GLOBAL_TASKS*2] = {");
			for(int v=1; v<(slaveProcessors*int.Parse(tbxMaxTasksSlave.Text)*2); v++) InitializeVectorsMaster.Append("{-1, -1, -1}, ");
			InitializeVectorsMaster.Append("{-1, -1, -1}};\n");
			InitializeVectorsMaster.AppendLine("");	
			InitializeVectorsMaster.Append("char task_terminated[MAX_GLOBAL_TASKS] = {");
			for(int v=1; v<(slaveProcessors*int.Parse(tbxMaxTasksSlave.Text)); v++) InitializeVectorsMaster.Append("-1, ");
			InitializeVectorsMaster.Append("-1 };\n");
			
			
			
			try
            {
                idFile = new StreamWriter(projectPath + "//build//InitializeVectorsMaster.h");
				
				idFile.WriteLine(InitializeVectorsMaster);
                idFile.Close();
            }
            catch (System.Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
            #endregion


            progressBar.Value += 10;
            lblProgress.Text = "Compiling sources...";

            /* Fires the tasks compilation */
            #region Tasks compilation

            string warning = "";
           
            this.Cursor = Cursors.Default;
            btnGenerate.Enabled = true;

            progressBar.Value += 10;
			
			Environment.CurrentDirectory = projectPath + "/build";
			
			string command = "make clean";
			runCommand(command);
			
			command = "make all > generation.log 2> error_warning.log";
			runCommand(command);
			Environment.CurrentDirectory = Environment.CurrentDirectory + "/../../../";
			
			/* Reads the warning log and searches for the string "error" */
			
			// caralho
			TextReader errorFinder;
			String error;
			errorFinder = new StreamReader(projectPath + "/build/error_warning.log");
			bool warnings = false;
			bool exit = false;

			while (((error = errorFinder.ReadLine()) != null) && (!exit)){
				
				warnings = true;
				
				if (System.Text.RegularExpressions.Regex.IsMatch(error, "error", System.Text.RegularExpressions.RegexOptions.IgnoreCase))
					exit = true;		
									
			}
			
			
            
            /*project.ReadLine();
            projectName = project.ReadLine();*/
			
//            if (shell.ExitCode == 0)
//            {
//                try
//                {
//                    /* Reads the warning report */
//                    warning = File.ReadAllText(workingDirectory + "//simulation//" + projectName + "//build//error_warning.log");
//                }
//                catch (System.Exception ex)
//                {
//                    MessageBox.Show(ex.ToString());
//                    return;
//                }
//            }
//            else
//            {	/* Compilation error */
//                try
//                {
//                    MessageBox.Show(File.ReadAllText(workingDirectory + "//simulation//" + projectName + "//build//error_warning.log"), "Generation error", MessageBoxButtons.OK, MessageBoxIcon.Error);
//                }
//                catch (System.Exception ex)
//                {
//                    MessageBox.Show(ex.ToString());
//                }
//
//                progressBar.Value = 0;
//                lblProgress.Text = "Ready";
//                return;
//            }
            #endregion

            progressBar.Value += 10;
            lblProgress.Text = "Generating repository...";

            /* Creates the task repository (repository.vhd) */
            #region Repository.vhd

            StreamWriter repositoryFile;
            StringBuilder[] objectCodes = new StringBuilder[ProjectTimeInsertedTasks.Count];     /* Stores the tasks object codes */
            TextReader code;
            string objFileName;     /* Object file name */
            int lineCount;          /* Number of lines in the task object file */
            int codeSize;           /* Task object code size in Bytes */

            /* Sets the first task object code start in the repositoty */
            int emptyStartLine = (24 * ProjectTimeInsertedTasks.Count + 1);
            int taskStartAddress = 200000 * 4;

            /* Creates the repository file */
            try
            {
                repositoryFile = new StreamWriter(projectPath + "/repository.vhd");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Could not find the path " + projectPath + "", "Generation error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
			repositoryFile.WriteLine("-------------------------------------------------------------------------------------");
            repositoryFile.WriteLine("--\tTasks Repository");
            repositoryFile.WriteLine("--\t\tContains the object codes of all tasks");
            repositoryFile.WriteLine("-------------------------------------------------------------------------------------");
            repositoryFile.WriteLine("--repository structure:");
            repositoryFile.WriteLine("--[/this structure is replicaded according the number of tasks]");
            repositoryFile.WriteLine("--number of tasks");
            repositoryFile.WriteLine("--task id");
            repositoryFile.WriteLine("--task code size");
            repositoryFile.WriteLine("--processor (ffffffff means dynamic allocation)");
            repositoryFile.WriteLine("--task code start address");
            repositoryFile.WriteLine("--[/this structure is replicaded according the number of tasks]");
            repositoryFile.WriteLine("--tasks codes");
            repositoryFile.WriteLine("-------------------------------------------------------------------------------------");
            repositoryFile.WriteLine("library IEEE;");
            repositoryFile.WriteLine("use IEEE.Std_Logic_1164.all;\n");
            repositoryFile.WriteLine("package memory_pack is");
            repositoryFile.WriteLine("\ttype ram is array (0 to 200000) of std_logic_vector(31 downto 0);\n");
            repositoryFile.WriteLine("\tsignal memory : ram := (");

            int repo_words = 0;
            int repo_words_code = 200000;
            int task_code_words = 0;


            /* Writes the number of tasks in the repository */
            repositoryFile.WriteLine("\t\tx\"" + ProjectTimeInsertedTasks.Count.ToString("x8") + "\",");
			
            /* Writes the tasks descriptors (defined in \Software\master\kernel.h as TaskPackage struct) */
            foreach (Task task in ProjectTimeInsertedTasks)
            {

                /* Writes the task id */
                if (task.CoreType == 1) /* task should run in a plasma processor */
                {
                    /* task should run in a mblite processor */
                    uint task_id = (uint)TaskIDs.IndexOf(task) | (uint)0x80000000;
                    repositoryFile.WriteLine("\t\tx\"" + task_id.ToString("x8") + "\",--id " + task.Text);
			    }
                else
                {
                    repositoryFile.WriteLine("\t\tx\"" + TaskIDs.IndexOf(task).ToString("x8") + "\",--id " + task.Text);
			    }


                /* Opens the task object code file */
                objFileName = task.Text;
                objFileName = objFileName.Remove(objFileName.IndexOf(".")) + "_" + TaskIDs.IndexOf(task) + ".txt";
				
				try
                {
                    code = new StreamReader(projectPath + "/build/" + objFileName);
					
                }
                catch (FileLoadException ex)
                {
                    MessageBox.Show("File " + ex.FileName + " not found.", "Generation error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    repositoryFile.Close();
                    return;
                }

                /* Reads the task object code and count the lines number */
                lineCount = 0;
                //MessageBox.Show(TaskIDs.IndexOf(task).ToString());
                objectCodes[TaskIDs.IndexOf(task)] = new StringBuilder();
                while ((line = code.ReadLine()) != null)
                {
                    if (lineCount == 0)
                    {
                        objectCodes[TaskIDs.IndexOf(task)].AppendLine("\t\tx\"" + line + "\",-- " + task.Text);
			        }
                    else
                    {
                        objectCodes[TaskIDs.IndexOf(task)].AppendLine("\t\tx\"" + line + "\",");
			        }
                    lineCount++;
                }
                code.Close();

                /* Sets the task size in bytes (each file line stores a 32 bits word) */
                codeSize = lineCount * 4;

                /* Writes the number of 32 bits words (size) */
                repositoryFile.WriteLine("\t\tx\"" + lineCount.ToString("x8") + "\",");
            
                String str_processor;
                uint processor;

                /* Writes the processor where the task must be allocated */
                if (task.Processor != -1)
                {
                    repositoryFile.WriteLine("\t\tx\"" + task.Processor.ToString("x8") + "\",--processor");
			    }
                else
                {
                    repositoryFile.WriteLine("\t\tx\"ffffffff\",--processor");
			    }

                taskStartAddress = taskStartAddress - codeSize;
                /* Writes the task address entry point in the repository */
                repositoryFile.WriteLine("\t\tx\"" + taskStartAddress.ToString("x8") + "\",--object code start address");
                
                int cont = 0;
                string depAux;
                dependence dp;
                taskList tl = tskList.Find(delegate(taskList t)
                {
                    return t.task == task.Text.ToUpper();	
                });
                if (tl != null)
                {
                    for (int i = 0; i < tl.depList.Count; i++)
                    {
                        dp = tl.depList[i];
                        if (cont < tl.depList.Count)
                        {
                            depAux = dp.task;
                            Task taux = TaskIDs.Find(delegate(Task t)
                            {
                                return t.Text.ToUpper() == depAux;
                            });
                            repositoryFile.WriteLine("\t\tx\"" + TaskIDs.IndexOf(taux).ToString("x8") + "\",");
                            repositoryFile.WriteLine("\t\tx\"" + dp.flits.ToString("x8") + "\",");
				        }
                        else
                        {
                            repositoryFile.WriteLine("\t\tx\"ffffffff\",");
                            repositoryFile.WriteLine("\t\tx\"ffffffff\",");
				        }
                        cont++;
                    }
                    for (int i = cont; i < 10; i++)
                    {
                        repositoryFile.WriteLine("\t\tx\"ffffffff\",");
                        repositoryFile.WriteLine("\t\tx\"ffffffff\",");
			        }
                }
				
                //code.Close();
            }
            for (int i = emptyStartLine; i < (taskStartAddress / 4); i++)
            {
                repositoryFile.WriteLine("\t\tx\"00000000\",");
            }

            /* Write the tasks object codes */
            for (int i = (ProjectTimeInsertedTasks.Count - 1); i >= 0; i--)
            {
                repositoryFile.Write(objectCodes[i]);
            }
          
			repositoryFile.WriteLine("\t\tothers => x\"00000000\");");
            repositoryFile.WriteLine("\tend memory_pack;");
            repositoryFile.Close();
            #endregion
			
			/* Creates the task repository for SystemC (repository.h) */
            #region Repository.h

            StreamWriter repository_h_File;
            StringBuilder[] objectCodes_without = new StringBuilder[TaskIDs.Count];     /* Stores the tasks object codes */
            
            /* Sets the first task object code start in the repositoty */
			emptyStartLine = (24 * TaskIDs.Count + 1);
            taskStartAddress = 200000 * 4;

            /* Creates the repository file */
            try
            {
                repository_h_File = new StreamWriter(projectPath + "/repository.h");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Could not find the path " + projectPath + "", "Generation error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
			repository_h_File.WriteLine("#ifndef _taskRepository");
            repository_h_File.WriteLine("#define _taskRepository");
            repository_h_File.WriteLine("");
            repository_h_File.WriteLine("#define REPO_SIZE 200000// 200000 Positions");
            repository_h_File.WriteLine("unsigned int repository[REPO_SIZE] = {");
            repository_h_File.WriteLine("");
            repo_words = 0;
            repo_words_code = 200000;
            task_code_words = 0;
			

            /* Writes the number of tasks in the repository */
            repository_h_File.WriteLine("\t0x" + ProjectTimeInsertedTasks.Count.ToString("x8") + ",");

            /* Writes the tasks descriptors (defined in \Software\master\kernel.h as TaskPackage struct) */
            foreach (Task task in TaskIDs)
            {

                /* Writes the task id */
                if (task.CoreType == 1) /* task should run in a plasma processor */
                {
                    /* task should run in a mblite processor */
                    uint task_id = (uint)TaskIDs.IndexOf(task) | (uint)0x80000000;
               		repository_h_File.WriteLine("\t0x" + task_id.ToString("x8") + ",//id " + task.Text);
                }
                else
                {
               		repository_h_File.WriteLine("\t0x" + TaskIDs.IndexOf(task).ToString("x8") + ",//id " + task.Text);
                }


                /* Opens the task object code file */
                objFileName = task.Text;
                objFileName = objFileName.Remove(objFileName.IndexOf(".")) + "_" + TaskIDs.IndexOf(task) + ".txt";
				
				try
                {
                    code = new StreamReader(projectPath + "/build/" + objFileName);
					
                }
                catch (FileLoadException ex)
                {
                    MessageBox.Show("File " + ex.FileName + " not found.", "Generation error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                /* Reads the task object code and count the lines number */
                lineCount = 0;
                objectCodes_without[TaskIDs.IndexOf(task)] = new StringBuilder();
                while ((line = code.ReadLine()) != null)
                {
                    if (lineCount == 0)
                    {
               			objectCodes_without[TaskIDs.IndexOf(task)].AppendLine("\t0x" + line + ",//" + task.Text);
                    }
                    else
                    {
               			objectCodes_without[TaskIDs.IndexOf(task)].AppendLine("\t0x" + line + ",");
                    }
                    lineCount++;
                }
                code.Close();

                /* Sets the task size in bytes (each file line stores a 32 bits word) */
                codeSize = lineCount * 4;

                /* Writes the number of 32 bits words (size) */
                repository_h_File.WriteLine("\t0x" + lineCount.ToString("x8") + ",");

                String str_processor;
                uint processor;

                /* Writes the processor where the task must be allocated */
                if (task.Processor != -1)
                {
                	repository_h_File.WriteLine("\t0x" + task.Processor.ToString("x8") + ",//processor");
                }
                else
                {
        			repository_h_File.WriteLine("\t0xffffffff,//processor");
                }

                taskStartAddress = taskStartAddress - codeSize;
                /* Writes the task address entry point in the repository */
                repository_h_File.WriteLine("\t0x" + taskStartAddress.ToString("x8") + ",//object code start address");

                int cont = 0;
                string depAux;
                dependence dp;
                taskList tl = tskList.Find(delegate(taskList t)
                {
                    return t.task == task.Text.ToUpper();	
                });
                if (tl != null)
                {
                    for (int i = 0; i < tl.depList.Count; i++)
                    {
                        dp = tl.depList[i];
                        if (cont < tl.depList.Count)
                        {
                            depAux = dp.task;
                            Task taux = TaskIDs.Find(delegate(Task t)
                            {
                                return t.Text.ToUpper() == depAux;
                            });
               				repository_h_File.WriteLine("\t0x" + TaskIDs.IndexOf(taux).ToString("x8") + ",");
							repository_h_File.WriteLine("\t0x" + dp.flits.ToString("x8") + ",");
                        }
                        else
                        {
               				repository_h_File.WriteLine("\t0xffffffff,");
							repository_h_File.WriteLine("\t0xffffffff,");
                        }
                        cont++;
                    }
                    for (int i = cont; i < 10; i++)
                    {
               			repository_h_File.WriteLine("\t0xffffffff,");
						repository_h_File.WriteLine("\t0xffffffff,");
                    }
                }
				
                //code.Close();
            }
            for (int i = emptyStartLine; i < (taskStartAddress / 4); i++)
            {
                repository_h_File.WriteLine("\t0x00000000,");
            }

            /* Write the tasks object codes */
            for (int i = (TaskIDs.Count - 1); i >= 0; i--)
            {
                repository_h_File.Write(objectCodes_without[i]);
            }
            repository_h_File.WriteLine("};");
            repository_h_File.WriteLine("");
            repository_h_File.WriteLine("#endif");
            repository_h_File.Close();
            #endregion
			
		    progressBar.Value += 10;
            lblProgress.Text = "Generating run time inserted appplications repository...";

            /* Creates the runtime inserted applications repositories */
            #region Partial Repositories

            StreamWriter prepositoryFile;
			StreamWriter prepository_h_File;
            TextReader pcode;
            string pobjFileName;     /* Object file name */
            int plineCount;          /* Number of lines in the task object file */
            int pcodeSize;           /* Task object code size in Bytes */
            StringBuilder[] pobjectCodes = new StringBuilder[TaskIDs.Count];     /* Stores the tasks object codes */
            int appindex = 0;

            try
            {
                prepositoryFile = new StreamWriter(projectPath + "/dynamic_apps.vhd");
				prepository_h_File = new StreamWriter(projectPath + "/dynamic_apps.h");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Could not find the path " + projectPath + "", "Generation error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            prepositoryFile.WriteLine("-------------------------------------------------------------------------------------");
            prepositoryFile.WriteLine("--\tTasks Repository");
            prepositoryFile.WriteLine("--\t\tContains the object codes of all tasks");
            prepositoryFile.WriteLine("-------------------------------------------------------------------------------------");
            prepositoryFile.WriteLine("--repository structure:");
            prepositoryFile.WriteLine("--[/this structure is replicaded according the number of tasks]");
            prepositoryFile.WriteLine("--number of tasks");
            prepositoryFile.WriteLine("--task id");
            prepositoryFile.WriteLine("--task code size");
            prepositoryFile.WriteLine("--processor (ffffffff means dynamic allocation)");
            prepositoryFile.WriteLine("--task code start address");
            prepositoryFile.WriteLine("--[/this structure is replicaded according the number of tasks]");
            prepositoryFile.WriteLine("--tasks codes");
            prepositoryFile.WriteLine("-------------------------------------------------------------------------------------");
            prepositoryFile.WriteLine("library IEEE;");
            prepositoryFile.WriteLine("use IEEE.Std_Logic_1164.all;\n");
            prepositoryFile.WriteLine("use work.memory_pack.all;\n");
            prepositoryFile.WriteLine("package dynamic_apps_pack is");
            if (RunTimeInsertedAppsTasks.Count != 0) prepositoryFile.WriteLine("\ttype repository_array is array (0 to " + RunTimeInsertedAppsTasks.Count + ") of ram;\n");
            else prepositoryFile.WriteLine("\ttype repository_array is array (0 to 1) of ram;");
			
			prepository_h_File.WriteLine("#ifndef _dynAppsRepository");
            prepository_h_File.WriteLine("#define _dynAppsRepository");
			prepository_h_File.WriteLine("#define NUMBER_OF_APPS " + RunTimeInsertedAppsTasks.Count);
            if (RunTimeInsertedAppsTasks.Count != 0)
            {
				aux = "";
                float pastTime = 0;
                float delay = 0;
                for (int i = 0; i < SortedApps.Count; i++)
                {
                    if (SortedApps[i].Value != 0)
                    {
                        delay = SortedApps[i].Value - pastTime;
                        aux = aux + delay.ToString().Replace(',', '.') + ",";
                        pastTime = SortedApps[i].Value;
                    }
                }
                aux = "{" + aux + "};";
                prepository_h_File.WriteLine("float appstime[" + RunTimeInsertedAppsTasks.Count + "] = " + aux);
            }
			else
			{
				prepository_h_File.WriteLine("float appstime[1] = { 0 };");
			}
            prepository_h_File.WriteLine("");
            
            foreach (KeyValuePair<TreeNode, float> element in SortedApps)
            {
                TreeNode tn = (TreeNode)element.Key;
				String app = tn.Text;
				float insertTime = (float)element.Value;
				if(insertTime!=0)
				{
						prepositoryFile.WriteLine("\n\t signal dynamic_app_" + appindex + " : ram := (");
						prepository_h_File.WriteLine("unsigned int dynamic_app_" + appindex + "[11] = {");
		
		                List<Task> AppTasks = (List<Task>)RunTimeInsertedAppsTasks[app];
		                /* Sets the first task object code start in the repositoty */
		                int ptaskStartAddress = (23 * AppTasks.Count + 11) * 4;
		
		                /* Creates the repository file */
		
		                /* Writes the number of tasks in the repository */
		                prepositoryFile.WriteLine("\t\tx\"" + AppTasks.Count.ToString("x8") + "\",");
						prepository_h_File.WriteLine("\t0x" + AppTasks.Count.ToString("x8") + ",");
		
		                /* Writes the application initial tasks */
		                int conta = 0;
		                List<Task> InitTasks = (List<Task>)AppsInitTasks[app];
		
		                for (int i = 0; (i < InitTasks.Count) && (i < 10); i++)
		                {
		                   // if (InitTasks[i].CoreType == 1)
		                   // {
		                        /* task should run in a mblite processor */
		                   /*     uint task_id = (uint)TaskIDs.IndexOf(InitTasks[i]) | (uint)0x80000000;
		                        prepositoryFile.WriteLine("\t\tx\"" + task_id.ToString("x8") + "\",--initial task id " + InitTasks[i].Text);
		                    }
		                    else
		                    {
		                      */  /* task should run in a plasma processor */
		                        prepositoryFile.WriteLine("\t\tx\"" + TaskIDs.IndexOf(InitTasks[i]).ToString("x8") + "\",--initial task id " + InitTasks[i].Text);
								prepository_h_File.WriteLine("\t0x" + TaskIDs.IndexOf(InitTasks[i]).ToString("x8") + ",//initial task id " + InitTasks[i].Text);
		                    //}
		                    conta++;
		                }
		                for (int i = conta; i < 10; i++)
		                {
		                    prepositoryFile.WriteLine("\t\tx\"ffffffff\",");
							prepository_h_File.WriteLine("\t0xffffffff,");
		                }
		
		                /* Writes the tasks descriptors (defined in \Software\master\kernel.h as TaskPackage struct) */
		                foreach (Task task in AppTasks)
		                {
		                    /* Writes the task id */
		                    if (task.CoreType == 1)
		                    {
		                        /* task should run in a mblite processor */
		                        uint task_id = (uint)TaskIDs.IndexOf(task) | (uint)0x80000000;
		                        prepositoryFile.WriteLine("\t\tx\"" + task_id.ToString("x8") + "\",--id " + task.Text);
		                    }
		                    else
		                    {
		                        /* task should run in a plasma processor */
		                        prepositoryFile.WriteLine("\t\tx\"" + TaskIDs.IndexOf(task).ToString("x8") + "\",--id " + task.Text);
		                    }
		
		
		                    /* Opens the task object code file */
		                    pobjFileName = task.Text;
		                    pobjFileName = pobjFileName.Remove(pobjFileName.IndexOf(".")) + "_" + TaskIDs.IndexOf(task) + ".txt";
		
		                    try
		                    {
		                        pcode = new StreamReader(projectPath + "//build//" + pobjFileName);
		                    }
		                    catch (FileLoadException ex)
		                    {
		                        MessageBox.Show("File " + ex.FileName + " not found.", "Generation error", MessageBoxButtons.OK, MessageBoxIcon.Error);
		                        prepositoryFile.Close();
		                        return;
		                    }
		
		                    /* Reads the task object code and count the lines number */
		                    plineCount = 0;
		                    pobjectCodes[TaskIDs.IndexOf(task)] = new StringBuilder();
		                    while ((line = pcode.ReadLine()) != null)
		                    {
		                        pobjectCodes[TaskIDs.IndexOf(task)].AppendLine("\t\tx\"" + line + "\",");
		                        plineCount++;
		                    }
		
		                    /* Sets the task size in bytes (each file line stores a 32 bits word) */
		                    pcodeSize = plineCount * 4;
		
		                    /* Writes the number of 32 bits words (size) */
		                    prepositoryFile.WriteLine("\t\tx\"" + plineCount.ToString("x8") + "\",");
		
		                    /* Writes the task address entry point in the repository */
		                    prepositoryFile.WriteLine("\t\tx\"" + ptaskStartAddress.ToString("x8") + "\",--object code start address");
		                    repo_words++;
		
		                    int cont = 0;
		                    string depAux;
		                    dependence dp;
		                    taskList tl = tskList.Find(delegate(taskList t)
		                    {
		                        return t.task == task.Text.ToUpper();
		                    });
		                    if (tl != null)
		                    {
		                        for (int i = 0; i < tl.depList.Count; i++)
		                        {
		                            dp = tl.depList[i];
		                            if (cont < tl.depList.Count)
		                            {
		                                depAux = dp.task;
		                                Task taux = TaskIDs.Find(delegate(Task t)
		                                {
		                                    return t.Text.ToUpper() == depAux;
		                                });
		                                prepositoryFile.WriteLine("\t\tx\"" + TaskIDs.IndexOf(taux).ToString("x8") + "\",");
		                                prepositoryFile.WriteLine("\t\tx\"" + dp.flits.ToString("x8") + "\",");
		                            }
		                            else
		                            {
		                                prepositoryFile.WriteLine("\t\tx\"ffffffff\",");
		                                prepositoryFile.WriteLine("\t\tx\"ffffffff\",");
		                            }
		                            cont++;
		                        }
		                        for (int i = cont; i < 10; i++)
		                        {
		                            prepositoryFile.WriteLine("\t\tx\"ffffffff\",");
		                            prepositoryFile.WriteLine("\t\tx\"ffffffff\",");
		                        }
		                    }
		
		                    ptaskStartAddress += pcodeSize;
		
		                    pcode.Close();
		                }
		
		                /* Write the tasks object codes */
		                foreach (StringBuilder sb in pobjectCodes)
		                    prepositoryFile.Write(sb);
		
		                prepository_h_File.WriteLine("};");
						prepositoryFile.WriteLine("\t\tothers => x\"00000000\");");
		                appindex++;
				}
            }
            if (RunTimeInsertedAppsTasks.Count != 0)
            {
                prepositoryFile.WriteLine("\tsignal dynamic_apps : repository_array := (");
				prepository_h_File.WriteLine("unsigned int dynamic_apps[" + RunTimeInsertedAppsTasks.Count + "][11];\n");
				prepository_h_File.WriteLine("void InitializeAppsInfo()");
				prepository_h_File.WriteLine("{");
                for (int i = 0; i < RunTimeInsertedAppsTasks.Count; i++)
                {
                    //prepository_h_File.WriteLine("\tt{*dynamic_app_" + i + "},");
					 for (int j = 0; j < 11; j++)
                	 {
						prepository_h_File.WriteLine("\tdynamic_apps[" + i + "][" + j + "] = dynamic_app_" + i + "[" + j + "];");	
					 }
					prepositoryFile.WriteLine("\t\tdynamic_app_" + i + ",");
                }
                prepository_h_File.WriteLine("}");
				prepositoryFile.WriteLine("\t\t(others =>(others=>'0'))");
                prepositoryFile.WriteLine("\t);");
            }
            else
            {
                prepository_h_File.WriteLine("unsigned int dynamic_apps[1][1] = { { 0 }, };");
				prepositoryFile.WriteLine("\tsignal dynamic_apps : repository_array := ( (others =>(others=>'0')), (others =>(others=>'0')));");
				prepository_h_File.WriteLine("void InitializeAppsInfo() {}");
            }
            prepositoryFile.WriteLine("end dynamic_apps_pack;");
			prepositoryFile.Close();
			
			prepository_h_File.WriteLine("");
            prepository_h_File.WriteLine("#endif");
            prepository_h_File.Close();
            #endregion

            progressBar.Value = 100;
            lblProgress.Text = "Ready";

            /* Compilation result  */
			if (exit)
				MessageBox.Show(File.ReadAllText(projectPath + "/build/error_warning.log"), "Generation error", MessageBoxButtons.OK, MessageBoxIcon.Error);			        
			        
            else if (!warnings)	/* No warnings */
                MessageBox.Show("HeMPS successfully generated.", "Success", MessageBoxButtons.OK);

            else if (MessageBox.Show("HeMPS successfully generated, but there are some compilation warnings.\nDo you want to see the warning report ?", "Success", MessageBoxButtons.YesNo, MessageBoxIcon.Information) == DialogResult.Yes)
                MessageBox.Show(File.ReadAllText(projectPath + "/build/error_warning.log"), "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);

            progressBar.Value = 0;
        }

        /* Starts the report window */
        private void btnDebug_Click(object sender, EventArgs e)
        {

            ProcessingElement peMaster = null;

            /* Gets the master coordinate */
            foreach (ProcessingElement pe in tlpProcessingElements.Controls)
                if (pe.Master)
                {
                    peMaster = pe;
                    break;
                }

            //fills_allocated_tasks_list();

		/* Verifies if the output_master file actually exists */
			
		StreamReader strReader;
        try
        {
            strReader = new StreamReader(new FileStream(projectPath + "/output_master.txt", FileMode.Open, FileAccess.Read, FileShare.ReadWrite));
				
			(new ReportWindow((int)nudX.Value, (int)nudY.Value, peMaster, TaskIDs, projectPath)).Show();
		}
			
		catch (FileNotFoundException ex)
        {
            MessageBox.Show("File " + ex.FileName + " not found.\nSimulate the generated HeMPS architecture before debug.", "Report error", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
			
			
			
            
        }

        /* Starts the drag operation */
        private void tvApplications_ItemDrag(object sender, ItemDragEventArgs e)
        {

            if (((TreeNode)(e.Item)).Level == 1) /* Drags only tasks */
                DoDragDrop(e.Item, DragDropEffects.Move);
        }

        /* New Master event handler */
        private void ProcessingElement_NewMaster(object source, EventArgs arg)
        {

            /* Sets the former master to slave and the tasks locations */
            foreach (ProcessingElement pe in processingElement)
                if (pe.Master && pe != (ProcessingElement)source)
                {
                    pe.Slave = true;

                    /* Removes the dynamic alocation, since the task was in the master processor */
                    foreach (Task t in pe.TaskList.Items)
                        t.Processor = pe.Address;

                    break;
                }
        }

        /* Saves the current architecture description as a project */
        private void tsmiSave_Click(object sender, EventArgs e)
        {
       		if(projectPath == null) SaveAs();
			else
			{
				Save(true);
			}
        }

        /* Restores an architecture description (project) */
        private void tsmiRestore_Click(object sender, EventArgs e)
        {

            RestoreProject();
        }

        /* New application */
        private void tsmiNew_Click(object sender, EventArgs e)
        {
        	if(!projectIsSaved && projectPath!=null)
			{
				DialogResult result1 = MessageBox.Show("Would you like to save the current project?", "HeMPS Information", MessageBoxButtons.YesNoCancel);
	            if (result1 == DialogResult.Yes) Save(true);
				
				if (result1 != DialogResult.Cancel)
				{
					NewProject();
				}
			}
			else
			{
				NewProject();
			}
		}

        /* Removes applications or tasks from the applications tree view */
        private void tvApplications_Delete_KeyDown(object sender, KeyEventArgs e)
        {

        }

        public void SetToZeroAppStartTime(String app)
        {
            AppStartTime[app] = "0 ms";
            foreach (ListViewItem lvi in lvAppStartTime.Items)
                if (lvi.Text == app) lvi.SubItems[1].Text = "0 ms";
        }

        public void SetToOneAppStartTime(String app)
        {
            bool ok = true;

            /* Search all processors */
            foreach (ProcessingElement pe in tlpProcessingElements.Controls)
            {
                /* Retrieves all allocated tasks */
                foreach (Task task in pe.TaskList.Items)
                    if (task.Application.Text == app) ok = false;
            }

            if (ok)
            {
                AppStartTime[app] = "1 ms";
                foreach (ListViewItem lvi in lvAppStartTime.Items)
                    if (lvi.Text == app) lvi.SubItems[1].Text = "1 ms";
            }
        }

        /* Creates a new project */
        void NewProject()
        {
            
			NewProject np;
			np = new NewProject();
	        np.ShowDialog();
	        np.Dispose();
			
			if(np.Result==DialogResult.OK)
			{
	           	projectName = np.NAme;
				projectPath = np.Path + "/" + projectName;
				
				/* Creates the directories */
				Directory.CreateDirectory(projectPath);
				Directory.CreateDirectory(projectPath+"/plasma_ram");
				Directory.CreateDirectory(projectPath+"/plasma_ram/sc");
				Directory.CreateDirectory(projectPath+"/plasma_ram/rtl");
				Directory.CreateDirectory(projectPath+"/mblite_ram");
				Directory.CreateDirectory(projectPath+"/mblite_ram/sc");
				Directory.CreateDirectory(projectPath+"/mblite_ram/rtl");
				Directory.CreateDirectory(projectPath+"/build");
				Directory.CreateDirectory(projectPath+"/log");
				Directory.CreateDirectory(projectPath+"/applications");
	
	            this.Text = "HeMPS Generator (Project - " + projectName + ")";
				
				AppStartTime.Clear();
		        lvAppStartTime.Items.Clear();
	
	            /* Removes all applications from the tree view */
	            tvApplications.Nodes.Clear();
	
	            /* Nodes initialization */
	            foreach (ProcessingElement pe in processingElement)
	            {
	                pe.TaskList.Items.Clear();  /* Remove allocated tasks */
	                pe.Slave = true;			/* Sets the processor as slave */
	            }
				/* Sets the initial master processor */
        		processingElement[0, 0].Master = true;
				
	            /* Repositories initialization */
	            foreach (Repository rep in repository)
	                rep.TaskList.Items.Clear();
				
	          	Save(false);
			}
		}

        /* Saving */
        void Save(bool showMessage)
        {
            SaveFileDialog sfdSaveProject = new SaveFileDialog();
            StreamWriter project;
            ProcessingElement peMaster = null;
            string directory = projectPath + "/" + projectName + ".hmp";
			
            /* Initialize the save project dialog */
            sfdSaveProject.RestoreDirectory = false;
            sfdSaveProject.AddExtension = true;
            sfdSaveProject.DefaultExt = ".hmp";
            sfdSaveProject.Filter = "HeMPS project (*.hmp)|*.hmp|All files (*.*)|*.*";
            sfdSaveProject.InitialDirectory = directory;
            sfdSaveProject.Title = "Save project as...";
            //sfdSaveProject.ShowDialog();
            sfdSaveProject.FileName = projectName;

            /* No selected file */
            /*if ( sfdSaveProject.FileName.Length == 0 )
                return;*/

            try
            {
                /* Creates the project file */
                project = new StreamWriter(directory);

                /* Saves the page size */
                project.WriteLine("[project name]");
                project.WriteLine(projectName);
				
				
				project.WriteLine("[page size]");
                project.WriteLine(dudPageSize.SelectedItem);

                /* Saves the memory size */
                project.WriteLine("[memory size]");
                if (rbtn64KB.Checked)
                    project.WriteLine("64");
                else
                    project.WriteLine("128");
				
				project.WriteLine("[processor description]");
				
				if(rbtnRtl.Checked)
					project.WriteLine("rtl");
				else if(rbtnIss.Checked)
					project.WriteLine("iss");
				else if(rbtnSc.Checked)
					project.WriteLine("sc");
				else if(rbtnScModelsim.Checked)
					project.WriteLine("scmod");

                /* Saves the Platform dimensions */
                project.WriteLine("[dimensions]");
                project.WriteLine(nudX.Value);
                project.WriteLine(nudY.Value);

                /* Searches for the master processor and writes the core for each processor */
                project.WriteLine("[processors core]");
                foreach (ProcessingElement pe in tlpProcessingElements.Controls)
                {
                    /* Gets the core */
                    if (pe.Plasma)
                        project.WriteLine("plasma");
                    else
                        project.WriteLine("mblite");

                    /* Sets the master processor */
                    if (pe.Master)
                        peMaster = pe;
                }

                /* Saves the master processor coordinades */
                project.WriteLine("[master processor]");
                if (peMaster == null)
                    project.WriteLine("null");
                else
                {
                    project.WriteLine(peMaster.X);
                    project.WriteLine(peMaster.Y);
                }

                /* Saves the application infos */
                if (tvApplications.Nodes.Count > 0)
                {
                    foreach (TreeNode tn in tvApplications.Nodes)
                    {
                        project.WriteLine("[application]");
                        project.WriteLine(tn.Text);	/* Saves the application name */
                        project.WriteLine(AppStartTime[tn.Text]);
                        project.WriteLine("[application directory]");
                        //project.WriteLine(tn.ToolTipText.Substring(tn.ToolTipText.IndexOf("//software//applications")));	/* Saves the application directory path relative to the working directory */
						project.WriteLine("applications/" + tn.Text);	/* Saves the application directory path relative to the working directory */
						
						/* Creates the application folder*/
						if (!Directory.Exists(projectPath + "/applications/" + tn.Text)){
							//MessageBox.Show("mkdir " + projectPath + "/applications/" + tn.Text);
							  Directory.CreateDirectory( projectPath + "/applications/" + tn.Text);
						}
						
						
                        /* Saves the application allocated tasks */
                        /* Searches the processors for allocated tasks */
                        project.WriteLine("[allocated tasks]");
                        foreach (ProcessingElement pe in processingElement)
                            foreach (Task task in pe.TaskList.Items)
                                if (task.Application == tn)
                                {	/* Saves the tasks properties */
                                    project.WriteLine(task.Text);
                                    project.WriteLine(pe.X);
                                    project.WriteLine(pe.Y);
                                    project.WriteLine(task.Priority);
                                    project.WriteLine(task.TimeSlice);
				
									/* Copy tasks to the user folder */
									if (!File.Exists(projectPath + "/applications/" + tn.Text + "/" + task.Text)){
										//MessageBox.Show("copy " + hemps_pathDirectory + "/applications/" + tn.Text + "/" + task.Text + " to " + projectPath + "/applications/" + tn.Text + "/" + task.Text);
									    	              File.Copy(hemps_pathDirectory + "/applications/" + tn.Text + "/" + task.Text,           projectPath + "/applications/" + tn.Text + "/" + task.Text);
									}
                                }

                        /* Searches the repositories for allocated tasks */
                        foreach (Repository rep in tlpRepository.Controls)
                            foreach (Task t in rep.TaskList.Items)
                                if (t.Application == tn)
                                {
                                    project.WriteLine(t.Text);
                                    project.WriteLine("-1"); //dynamic allocation
                                    if (rep.Repository_Type == 0) // plasma repository
                                        project.WriteLine("0");
                                    else                          // mblite repository
                                        project.WriteLine("1");
                                    project.WriteLine(t.Priority);
									project.WriteLine(t.TimeSlice);
									if (!File.Exists(projectPath + "/applications/" + tn.Text + "/" + t.Text)){
										//MessageBox.Show("copy " + hemps_pathDirectory + "/applications/" + tn.Text + "/" + t.Text + " to " + projectPath + "/applications/" + tn.Text + "/" + t.Text);
										                  File.Copy(hemps_pathDirectory + "/applications/" + tn.Text + "/" + t.Text,           projectPath + "/applications/" + tn.Text + "/" + t.Text);
                                    }
                                }

                        /* Saves the application not allocated tasks */
                        project.WriteLine("[not allocated tasks]");
                        foreach (TreeNode tn2 in tn.Nodes)
                            project.WriteLine(tn2.Text);

                        project.WriteLine("[end application]");
                    }
                }				

                /* Finishes the save */
                project.WriteLine("[end]");

                if (showMessage)
                    MessageBox.Show("Project successfully saved.", "Success", MessageBoxButtons.OK);

                /* Sets the Window title with the project name */
                this.Text = "HeMPS Generator (Project - " + sfdSaveProject.FileName + ")";
					
				projectIsSaved = true;
					
                project.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }


        }
		
		void SaveAs()
        {
            
			NewProject np;
			np = new NewProject();
	        np.ShowDialog();
	        np.Dispose();
			
			if(np.Result==DialogResult.OK)
			{
	           	projectName = np.NAme;
				projectPath = np.Path + "/" + projectName;
				
				/* Creates the directories */
				Directory.CreateDirectory(projectPath);
				Directory.CreateDirectory(projectPath+"/plasma_ram");
				Directory.CreateDirectory(projectPath+"/plasma_ram/sc");
				Directory.CreateDirectory(projectPath+"/plasma_ram/rtl");
				Directory.CreateDirectory(projectPath+"/mblite_ram");
				Directory.CreateDirectory(projectPath+"/mblite_ram/sc");
				Directory.CreateDirectory(projectPath+"/mblite_ram/rtl");
				Directory.CreateDirectory(projectPath+"/build");
				Directory.CreateDirectory(projectPath+"/log");
				Directory.CreateDirectory(projectPath+"/applications");
	
	            this.Text = "HeMPS Generator (Project - " + projectName + ")";
			
				Save(false);
			}
		}
		

        /* Function to restore the project
         * returns a value to show if the loading was sucessful or not*/
        bool RestoreProject()
        {

            OpenFileDialog ofdRestoreProject = new OpenFileDialog();
            TextReader project;
            string str;
            Task task;
            TreeNode tnApplication;
            int x, y;
			String[] split;
			String path, processor_description;
			
			
			/* Gets the first two folders in the current directory (home2/something) */
			split = Environment.CurrentDirectory.Split('/');
			path = "/"+ split[1] + "/" + split[2];	
			
			
            /* Initializes the restore project dialog */
            ofdRestoreProject.Filter = "HeMPS project (*.hmp)|*.hmp|All files (*.*)|*.*";
            ofdRestoreProject.InitialDirectory = path;
            ofdRestoreProject.Title = "Restore project";
            ofdRestoreProject.ShowDialog();
			
			
	        /* No selected file */
            if (ofdRestoreProject.FileName.Length == 0)
                return false;

            try
            {
                /* Open the project file */
                project = new StreamReader(ofdRestoreProject.FileName);

                /* Reads [project name]*/
                project.ReadLine();
                projectName = project.ReadLine();
				
				
				projectPath = System.IO.Path.GetDirectoryName(ofdRestoreProject.FileName);

	            /* Sets the Window title with the project name */
                this.Text = "HeMPS Generator (Project - " + projectName + ")";

                /* Removes all applications from the tree view */
                tvApplications.Nodes.Clear();

                AppStartTime.Clear();

                /* Nodes initialization */
                foreach (ProcessingElement pe in processingElement)
                {
                    pe.TaskList.Items.Clear();  /* Remove allocated tasks */
                    pe.Slave = true;			/* Sets the processor as slave */
                }

                /* Repositories initialization */
                foreach (Repository rep in repository)
                    rep.TaskList.Items.Clear();



                /* Reads [page size] */
                project.ReadLine();
                dudPageSize.SelectedItem = project.ReadLine();

                /* Reads [memory size] */
                project.ReadLine();
                if (project.ReadLine().Equals("64"))
                    rbtn64KB.Checked = true;
                else
                    rbtn128KB.Checked = true;

				/* Reads [processor description] */
				project.ReadLine();
				processor_description = project.ReadLine();
				
				/* Selects the processor description */
				if (processor_description == "rtl")
					rbtnRtl.Checked = true;
				
				else if (processor_description == "iss")
					rbtnIss.Checked = true;
				
				else if (processor_description == "sc")
					rbtnSc.Checked = true;				
				
				else if (processor_description == "scmod")
					rbtnScModelsim.Checked = true;	
							
				

                /* Reads [dimensions] */
                project.ReadLine();
                nudX.Value = Convert.ToDecimal(project.ReadLine());
                nudY.Value = Convert.ToDecimal(project.ReadLine());

                /* Adds processors in the processors panel (nodes_panel) */
                /*for (y = 0; y < nudY.Value; y++)
                    for (x = 0; x < nudX.Value; x++)
                    {
                        processingElement[x, y].X = x;
                        processingElement[x, y].Y = y;
                        processingElement[x, y].Address = NodeAddress(x, y);
                        //tlpProcessingElements.Controls.Add(processingElement[x, y], x, (int)nudY.Value - y - 1);
                    }*/

                /* Reads [processors core] */
                project.ReadLine();
                for (y = 0; y < nudY.Value; y++)
                    for (x = 0; x < nudX.Value; x++)
                    {
                        processingElement[x, y].X = x;
                        processingElement[x, y].Y = y;
                        processingElement[x, y].Address = NodeAddress(x, y);

                        if (project.ReadLine() == "plasma")
                            processingElement[x, y].Plasma = true;
                        else
                            processingElement[x, y].MBLite = true;

                        tlpProcessingElements.Controls.Add(processingElement[x, y], x, (int)nudY.Value - y - 1);
                    }

                /*foreach (ProcessingElement pe in tlpProcessingElements.Controls)
                    if (project.ReadLine() == "plasma")
                        pe.Plasma = true;
                    else
                        pe.MBLite = true;
                */

                /* Reads the master processor */
                if ((str = project.ReadLine()).Equals("[master processor]"))
                {
                    x = Convert.ToUInt16(project.ReadLine());
                    y = Convert.ToUInt16(project.ReadLine());
                    processingElement[x, y].Master = true;
                }

                /* Reads the applications infos */
                while (!project.ReadLine().Equals("[end]"))
                { /* Reads [application] or [end] */

                    /* Reads the application name */
                    tnApplication = new TreeNode(project.ReadLine());

                    /* Reads the application start time */
                    String StartTime = project.ReadLine();
                    AppStartTime.Add(tnApplication.Text, StartTime);
                    /* Adds application to the lvAppStartTime */
                    ListViewItem lvi = new ListViewItem(tnApplication.Text);
                    lvi.SubItems.Add(StartTime);
                    lvAppStartTime.Items.Add(lvi);

                    /* Reads [application directory] */
                    project.ReadLine();
                    tnApplication.ToolTipText = project.ReadLine();

                    /* Adds the application in the tree view */
                    tvApplications.Nodes.Add(tnApplication);

                    /* Reads [allocated tasks] */
                    project.ReadLine();
                    while (!(str = project.ReadLine()).Equals("[not allocated tasks]"))
                    {
                        x = Convert.ToInt32(project.ReadLine());
                        y = Convert.ToInt32(project.ReadLine());

                        /*x = Convert.ToUInt16(project.ReadLine());
                        y = Convert.ToUInt16(project.ReadLine());*/
                        if (x != -1)
                        {
                            x = Convert.ToUInt16(x);
                            y = Convert.ToUInt16(y);
                            task = new Task(str, processingElement[x, y].Address);  /* Task staticaly allocated */
                        }
                        else
                            task = new Task(str, -1);   /* Task dynamic allocated */

                        task.Priority = Convert.ToInt32(project.ReadLine());
                        task.TimeSlice = Convert.ToInt32(project.ReadLine());

                        task.Application = tnApplication;	/* Sets the task application */
                        task.ToolTipText = ((TreeNode)tnApplication).Text;

                        if (x != -1)
                            processingElement[x, y].TaskList.Items.Add(task);	/* Adds the task in the processor */
                        else
                            if (y == 0)                             /* Adds the task in the repository */
                                repository[0].TaskList.Items.Add(task);         /* Plasma task in the plasma repository */
                            else
                                repository[1].TaskList.Items.Add(task);         /* MBLite task in the mblite repository */
                    }

                    /* Reads the not allocated tasks */
                    while (!(str = project.ReadLine()).Equals("[end application]"))
                        tnApplication.Nodes.Add(str);	/* Adds the tasks in the application */
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Corromped project file.", "Restore error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return false;
            }

            tvApplications.ExpandAll();
            //MessageBox.Show("Project successfully restored.", "Success", MessageBoxButtons.OK);

            project.Close();

            return true;
        }

        void fills_allocated_tasks_list()
        {
            for (int y = 0; y < nudY.Value; y++)
                for (int x = 0; x < nudX.Value; x++)
                {
                    foreach (Task task in processingElement[x, y].TaskList.Items)
                    {
                        if (processingElement[x, y].Plasma)
                            task.CoreType = 0;
                        else
                            task.CoreType = 1;

                        lstAllocatedTasks.Add(task);
                    }
                }

            for (int i = 0; i < 2; i++)
                foreach (Task task in repository[i].TaskList.Items)
                    lstAllocatedTasks.Add(task);
        }

        private void txtbSimDuration_TextChanged(object sender, EventArgs e)
        {

            string text = (sender as TextBox).Text;

            StringBuilder builder = new StringBuilder(String.Empty);

            foreach (char character in text)
            {
                if (Char.IsDigit(character))
                {
                    builder.Append(character);
                }
            }

            (sender as TextBox).Text = builder.ToString();
        }

        private void bChangeStartTime_Click(object sender, EventArgs e)
        {
            List<String> lstAppsInPEs = new List<String>();
            List<String> lstAppsInRep = new List<String>();
            int error = 0;

            foreach (ProcessingElement pe in tlpProcessingElements.Controls)
            {
                foreach (Task task in pe.TaskList.Items)
                {
                    if (!lstAppsInPEs.Contains(task.Application.Text)) lstAppsInPEs.Add(task.Application.Text);
                }
            }

            foreach (Repository rep in tlpRepository.Controls)
                foreach (Task t in rep.TaskList.Items)
                {
                    lstAppsInRep.Add(t.Application.Text);
                }

            foreach (ListViewItem item in lvAppStartTime.Items)
            {
                if (item.Checked == true)
                {
                    error = 0;
                    if (lstAppsInPEs.Contains(item.SubItems[0].Text) && float.Parse(tbStartTime.Text) > 0)
                    {
                        MessageBox.Show("The start time of the application " + item.SubItems[0].Text + " was not changed, because it has tasks allocated for static mapping.", "Alert", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        error = 1;
                    }
                    if (float.Parse(tbStartTime.Text) < 0)
                    {
                        MessageBox.Show("The start time of the application " + item.SubItems[0].Text + " was not changed, because you choose a negative start time.", "Alert", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        error = 1;
                    }
                    if (lstAppsInRep.Contains(item.SubItems[0].Text) && float.Parse(tbStartTime.Text) == 0)
                    {
                        MessageBox.Show("The start time of the application " + item.SubItems[0].Text + " was not changed, because if you want to set its value to 0, you need to allocate tasks for static mapping.", "Alert", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        error = 1;
                    }

                    if (error == 0)
                    {
                        item.SubItems[1].Text = tbStartTime.Text + " " + cbUnit.Text;
                        AppStartTime[item.SubItems[0].Text] = tbStartTime.Text + " " + cbUnit.Text;
                    }
                }
            }

        }

        private void tbStartTime_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (!System.Text.RegularExpressions.Regex.IsMatch(e.KeyChar.ToString(), "[0-9.,]") && char.IsControl(e.KeyChar) == false)
                e.Handled = true;
        }

        private void tvApplications_AfterSelect(object sender, TreeViewEventArgs e)
        {
            /*  Finds the tasks in the processors */
            foreach (ProcessingElement pe in processingElement)
                pe.SelectsTasks(tvApplications.SelectedNode);

            foreach (Repository rep in repository)
                rep.SelectsTasks(tvApplications.SelectedNode);
        }

        private void Generator_FormClosing(object sender, FormClosingEventArgs e)
        {
            if(projectPath!=null)
			{
				DialogResult result1 = MessageBox.Show("Would you like to save your project?", "HeMPS Information", MessageBoxButtons.YesNoCancel);
            	if (result1 == DialogResult.Yes) Save(true);
				if (result1 == DialogResult.Cancel) e.Cancel = true;
			}
	    }

        private void btnSimulate_Click(object sender, EventArgs e)
        {
			string command;
			
            /* disables the cursor and the simulate button */
            this.Cursor = Cursors.WaitCursor;
            btnSimulate.Enabled = false;
			
			
			/* Reads/Edits the file sim.do*/
			System.Text.StringBuilder sim_do = new System.Text.StringBuilder();
            StreamWriter sim_do_wr;
            
            try
            {
                //sim_do.AppendLine("make all");
                sim_do.AppendLine("vsim -novopt -t ps +notimingchecks work.test_bench"); //-Gmlite_description=ISS -Gram_description=ISS 
                sim_do.AppendLine("");
                sim_do.AppendLine("do wave.do");
                sim_do.AppendLine("onerror {resume}");
                sim_do.AppendLine("radix hex");
                sim_do.AppendLine("set NumericStdNoWarnings 1");
                sim_do.AppendLine("set StdArithNoWarnings 1");
                sim_do.AppendLine("");
                sim_do.AppendLine("when -label end_of_simulation { HeMPS/proc(" + masterAddressGlobal + ")/mas/master/PE_PLASMA/plasma/end_sim_reg == x\"00000000\" } {echo \"End of simulation\" ; quit ;}");
                sim_do.AppendLine("run " + txtbSimDuration.Text + " " + dudTime.Text);

                /* Writes the new Makefile file */
                sim_do_wr = new StreamWriter(projectPath + "//sim.do");
                sim_do_wr.Write(sim_do.ToString());
                sim_do_wr.Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
                return;
            }
			

            /* fires the sim.do */
			Environment.CurrentDirectory = projectPath + "/";
			
			command = "make clean";
			runCommand(command);
			
			
			command = "make all";
			runCommand(command);
			
			if(rbtnSc.Checked){
				command = "./HeMPS.exe -c " + txtbSimDuration.Text;
			}
			
			else{
				command = "vsim -do sim.do";
			}
			runCommand(command);
			Environment.CurrentDirectory = Environment.CurrentDirectory + "/../../";
			

            /* enables the cursor and the button back */
            this.Cursor = Cursors.Default;
            btnSimulate.Enabled = true;
        }

        private void tsmiSaveAs_Click(object sender, EventArgs e)
        {
            SaveAs();
        }

        private void aboutToolStripMenuItem_Click(object sender, EventArgs e)
        {
            (new AboutBox1()).Show();
        }

    }
}

class dependence
{
    public string task { get; set; }
    public int flits { get; set; }

    // Default constructor:
    public dependence()
    {
        task = "";
        flits = 0;
    }

    // Constructor:
    public dependence(string task, int flits)
    {
        this.task = task;
        this.flits = flits;
    }
}

class taskList
{
    public string task { get; set; }
    public List<dependence> depList { get; set; }

    // Default constructor:
    public taskList()
    {
        task = "";
        depList = new List<dependence>();
    }

    // Constructor:
    public taskList(string task, string application)
    {
        this.task = task;
        depList = new List<dependence>();
    }

    public void removeDuplicates()
    {
        Dictionary<string, int> uniqueStore = new Dictionary<string, int>();
        List<dependence> finalList = new List<dependence>();
        foreach (dependence currValue in depList)
        {
            if (!uniqueStore.ContainsKey(currValue.task))
            {
                uniqueStore.Add(currValue.task, 0);
                finalList.Add(currValue);
            }
        }
        depList = finalList;
    }

}

