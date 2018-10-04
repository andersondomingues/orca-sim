using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.IO;
using System.Text;
using System.Windows.Forms;

namespace HeMPS_Generator
{
    public partial class ReportWindow : Form
    {

        public ReportWindow(int columns, int rows, ProcessingElement peMaster, List<Task> TaskIDs, string project_name)
        {

            /* Controls */
            Report[,] report = new Report[columns, rows]; 
            TabPage[] tpgTask = new TabPage[TaskIDs.Count];
            TextBox[] txtTask = new TextBox[TaskIDs.Count];
            TabPage tpgKernelMaster = null;
            TextBox txtKernelMaster = null;

            StreamReader strReader;
            Task[] taskName = TaskIDs.ToArray();
            int processor, task, address;
            string line;
            string[] split;
            Point point = new Point();

            InitializeComponent();

            tlpReport.ColumnCount = columns;
            tlpReport.RowCount = rows;

            #region assigning_control_tabs_for_the_processors

            /* Adds the control tabs for each processor */
            for (int y = 0; y < rows; y++)
                for (int x = 0; x < columns; x++)
                {
                    if (peMaster.X == x && peMaster.Y == y)
                        report[x, y] = new Report("Processor " + peMaster.Address + " (Master)");
                    else
                    {
                        /* Converts coordinates address to Hamiltonian address */
                        //if (y % 2 == 0)	/* Even line */
                        /* first column */
                        address = 10 * x + y;
                        //address = y * rows + x;
                        //else			/* Odd line */
                        //    address =  y * columns + columns - x - 1;

                        /* first column */
                        if (x == 0)
                            report[x, y] = new Report("Processor " + 0 + address);
                        else
                            report[x, y] = new Report("Processor " + address);
                    }

                    tlpReport.Controls.Add(report[x, y], x, rows - y - 1);
                }
            #endregion



            /* Reads the output_master.txt file */
            #region reading_file
            try
            {
                strReader = new StreamReader(new FileStream(project_name + "/output_master.txt", FileMode.Open, FileAccess.Read, FileShare.ReadWrite));

                /* Reads a line from the file */
                while ((line = strReader.ReadLine()) != null)
                {

                    if (line.StartsWith("$"))
                    {	/* Slave line */

                        /* Splits the line */
                        split = line.Split(',');

                        /* Reads the source processor */
                        /* Converts string split[1] to decimal integer */
                        int aux = Convert.ToInt32(split[1]);
                        
                        /* Converts this decimal integer to an hexadecimal string */
                        string proc = aux.ToString("X");

                        /* Converts back to integer */
                        aux = Convert.ToInt32(proc);

                        processor = aux;

                        task = Convert.ToInt32(split[2]);


                        /* Creates a tab page and a text box for the task */
                        if (tpgTask[task] == null)
                        {
                            /* Creates the text box */
                            txtTask[task] = new TextBox();
                            txtTask[task].Multiline = true;
                            txtTask[task].ReadOnly = true;
                            txtTask[task].WordWrap = false;
                            txtTask[task].Size = new Size(report[0, 0].Tab.Width - 8, report[0, 0].Tab.Height - 26);
                            txtTask[task].ScrollBars = ScrollBars.Both;

                            /* Creates the tab page abd adds the text box */
                            tpgTask[task] = new TabPage(taskName[task].Text + " (" + task + ")");
                            tpgTask[task].Controls.Add(txtTask[task]);

                            /* Gets the coordinates XY of the processor */

                            point.X = aux / 10;
                            point.Y = aux % 10;

                            /* Adds the tab page in the control tab */
                            report[point.X, point.Y].Tab.Controls.Add(tpgTask[task]);
                        }

                        /* Adds the text in the text box */
                        txtTask[task].AppendText(split[3]);

                        /* In case of commas in the text */
                        if (split.Length > 3)
                            for (int i = 4; i < split.Length; i++)
                                txtTask[task].AppendText(',' + split[i]);

                        txtTask[task].AppendText("\r\n");
                    }
                    /* Master line */
                    else
                    {
                        /* Creates a tab page and a text box for the task */
                        if (tpgKernelMaster == null)
                        {
                            /* Creates the text box */
                            txtKernelMaster = new TextBox();
                            txtKernelMaster.Multiline = true;
                            txtKernelMaster.ReadOnly = true;
                            txtKernelMaster.WordWrap = false;
                            txtKernelMaster.Size = new Size(report[0, 0].Tab.Width - 8, report[0, 0].Tab.Height - 26);
                            txtKernelMaster.ScrollBars = ScrollBars.Both;

                            /* Creates the tab page and adds the text box */
                            tpgKernelMaster = new TabPage("Kernel");
                            tpgKernelMaster.Controls.Add(txtKernelMaster);

                            /* Adds the tab page in the control tab */
                            report[peMaster.X, peMaster.Y].Tab.Controls.Add(tpgKernelMaster);
                        }

                        /* Adds the text in the text box */
                        if (line.Length > 0)
                            txtKernelMaster.AppendText(line.Remove(0, 1) + "\r\n");
                    }
                }
            }
            catch (FileNotFoundException ex)
            {
                MessageBox.Show("File " + ex.FileName + " not found.\nSimulate the generated HeMPS architecture before debug.", "Report error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            catch (IndexOutOfRangeException ex)
            {
                MessageBox.Show("Simulation result does not corresponds to the current project.\nGenerate a new HeMPS architcture and simulate it before debug.", "Report error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            /* Generic exception */
            catch (Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }

            #endregion

        }
    }
}
