namespace HeMPS_Generator
{
    partial class Report
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
            this.tabControl = new System.Windows.Forms.TabControl();
            this.gbxProcessor.SuspendLayout();
            this.SuspendLayout();
            // 
            // gbxProcessor
            // 
            this.gbxProcessor.Controls.Add(this.tabControl);
            this.gbxProcessor.Location = new System.Drawing.Point(0, 0);
            this.gbxProcessor.Name = "gbxProcessor";
            this.gbxProcessor.Size = new System.Drawing.Size(264, 160);
            this.gbxProcessor.TabIndex = 0;
            this.gbxProcessor.TabStop = false;
            // 
            // tabControl
            // 
            this.tabControl.Location = new System.Drawing.Point(6, 17);
            this.tabControl.Name = "tabControl";
            this.tabControl.SelectedIndex = 0;
            this.tabControl.Size = new System.Drawing.Size(252, 137);
            this.tabControl.TabIndex = 0;
            // 
            // Report
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.gbxProcessor);
            this.Name = "PEReport";
            this.Size = new System.Drawing.Size(265, 161);
            this.gbxProcessor.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.GroupBox gbxProcessor;
        private System.Windows.Forms.TabControl tabControl;
    }
}
