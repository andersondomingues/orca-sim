namespace HeMPS_Generator
{
    partial class ProcessingElement
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.gbxProcessor = new System.Windows.Forms.GroupBox();
            this.panel1 = new System.Windows.Forms.Panel();
            this.rbtnPlasma = new System.Windows.Forms.RadioButton();
            this.rbtnMBLite = new System.Windows.Forms.RadioButton();
            this.btnRemove = new System.Windows.Forms.Button();
            this.rbtnSlave = new System.Windows.Forms.RadioButton();
            this.rbtnMaster = new System.Windows.Forms.RadioButton();
            this.lvTasks = new System.Windows.Forms.ListView();
            this.gbxProcessor.SuspendLayout();
            this.panel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // gbxProcessor
            // 
            this.gbxProcessor.AutoSize = true;
            this.gbxProcessor.BackColor = System.Drawing.Color.Transparent;
            this.gbxProcessor.Controls.Add(this.panel1);
            this.gbxProcessor.Controls.Add(this.btnRemove);
            this.gbxProcessor.Controls.Add(this.rbtnSlave);
            this.gbxProcessor.Controls.Add(this.rbtnMaster);
            this.gbxProcessor.Controls.Add(this.lvTasks);
            this.gbxProcessor.Location = new System.Drawing.Point(3, 3);
            this.gbxProcessor.Name = "gbxProcessor";
            this.gbxProcessor.Size = new System.Drawing.Size(214, 131);
            this.gbxProcessor.TabIndex = 0;
            this.gbxProcessor.TabStop = false;
            this.gbxProcessor.Enter += new System.EventHandler(this.gbxProcessor_Enter);
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.Transparent;
            this.panel1.Controls.Add(this.rbtnPlasma);
            this.panel1.Controls.Add(this.rbtnMBLite);
            this.panel1.Location = new System.Drawing.Point(139, 19);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(69, 64);
            this.panel1.TabIndex = 7;
            // 
            // rbtnPlasma
            // 
            this.rbtnPlasma.AutoSize = true;
            this.rbtnPlasma.Checked = true;
            this.rbtnPlasma.Location = new System.Drawing.Point(10, 10);
            this.rbtnPlasma.Name = "rbtnPlasma";
            this.rbtnPlasma.Size = new System.Drawing.Size(59, 17);
            this.rbtnPlasma.TabIndex = 6;
            this.rbtnPlasma.TabStop = true;
            this.rbtnPlasma.Text = "Plasma";
            this.rbtnPlasma.UseVisualStyleBackColor = true;
            this.rbtnPlasma.CheckedChanged += new System.EventHandler(this.rbtnPlasma_CheckedChanged);
            // 
            // rbtnMBLite
            // 
            this.rbtnMBLite.AutoSize = true;
            this.rbtnMBLite.Location = new System.Drawing.Point(10, 39);
            this.rbtnMBLite.Name = "rbtnMBLite";
            this.rbtnMBLite.Size = new System.Drawing.Size(58, 17);
            this.rbtnMBLite.TabIndex = 5;
            this.rbtnMBLite.Text = "MBLite";
            this.rbtnMBLite.UseVisualStyleBackColor = true;
            this.rbtnMBLite.CheckedChanged += new System.EventHandler(this.rbtnPlasma_CheckedChanged);
            // 
            // btnRemove
            // 
            this.btnRemove.Location = new System.Drawing.Point(110, 89);
            this.btnRemove.Name = "btnRemove";
            this.btnRemove.Size = new System.Drawing.Size(75, 23);
            this.btnRemove.TabIndex = 4;
            this.btnRemove.Text = "Remove";
            this.btnRemove.UseVisualStyleBackColor = true;
            this.btnRemove.Click += new System.EventHandler(this.btnRemove_Click);
            // 
            // rbtnSlave
            // 
            this.rbtnSlave.AutoSize = true;
            this.rbtnSlave.Checked = true;
            this.rbtnSlave.Location = new System.Drawing.Point(86, 58);
            this.rbtnSlave.Name = "rbtnSlave";
            this.rbtnSlave.Size = new System.Drawing.Size(52, 17);
            this.rbtnSlave.TabIndex = 3;
            this.rbtnSlave.TabStop = true;
            this.rbtnSlave.Text = "Slave";
            this.rbtnSlave.UseVisualStyleBackColor = true;
            this.rbtnSlave.CheckedChanged += new System.EventHandler(this.rbtnMaster_CheckedChanged);
            // 
            // rbtnMaster
            // 
            this.rbtnMaster.AutoSize = true;
            this.rbtnMaster.Location = new System.Drawing.Point(86, 29);
            this.rbtnMaster.Name = "rbtnMaster";
            this.rbtnMaster.Size = new System.Drawing.Size(57, 17);
            this.rbtnMaster.TabIndex = 2;
            this.rbtnMaster.Text = "Master";
            this.rbtnMaster.UseVisualStyleBackColor = true;
            this.rbtnMaster.CheckedChanged += new System.EventHandler(this.rbtnMaster_CheckedChanged);
            // 
            // lvTasks
            // 
            this.lvTasks.AllowDrop = true;
            this.lvTasks.Location = new System.Drawing.Point(6, 19);
            this.lvTasks.Name = "lvTasks";
            this.lvTasks.ShowItemToolTips = true;
            this.lvTasks.Size = new System.Drawing.Size(74, 92);
            this.lvTasks.TabIndex = 0;
            this.lvTasks.UseCompatibleStateImageBehavior = false;
            this.lvTasks.View = System.Windows.Forms.View.List;
            this.lvTasks.DragDrop += new System.Windows.Forms.DragEventHandler(this.lvTasks_DragDrop);
            this.lvTasks.DragEnter += new System.Windows.Forms.DragEventHandler(this.lvTasks_DragEnter);
            this.lvTasks.KeyDown += new System.Windows.Forms.KeyEventHandler(this.Delete_KeyDown);
            this.lvTasks.ItemDrag += new System.Windows.Forms.ItemDragEventHandler(this.lvTasks_ItemDrag);
            // 
            // ProcessingElement
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.Controls.Add(this.gbxProcessor);
            this.Name = "ProcessingElement";
            this.Size = new System.Drawing.Size(220, 137);
            this.gbxProcessor.ResumeLayout(false);
            this.gbxProcessor.PerformLayout();
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.GroupBox gbxProcessor;
        private System.Windows.Forms.ListView lvTasks;
        private System.Windows.Forms.RadioButton rbtnSlave;
        private System.Windows.Forms.RadioButton rbtnMaster;
        private System.Windows.Forms.Button btnRemove;
        private System.Windows.Forms.RadioButton rbtnMBLite;
        private System.Windows.Forms.RadioButton rbtnPlasma;
        private System.Windows.Forms.Panel panel1;
    }
}
