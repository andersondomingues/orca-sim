namespace HeMPS_Generator
{
    partial class Intro
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

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.newProject = new System.Windows.Forms.Button();
            this.loadProject = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // newProject
            // 
            this.newProject.Location = new System.Drawing.Point(51, 28);
            this.newProject.Name = "newProject";
            this.newProject.Size = new System.Drawing.Size(170, 27);
            this.newProject.TabIndex = 0;
            this.newProject.Text = "Create a New Project";
            this.newProject.UseVisualStyleBackColor = true;
            this.newProject.Click += new System.EventHandler(this.newProject_Click);
            // 
            // loadProject
            // 
            this.loadProject.Location = new System.Drawing.Point(51, 75);
            this.loadProject.Name = "loadProject";
            this.loadProject.Size = new System.Drawing.Size(170, 27);
            this.loadProject.TabIndex = 1;
            this.loadProject.Text = "Load an Existing Project";
            this.loadProject.UseVisualStyleBackColor = true;
            this.loadProject.Click += new System.EventHandler(this.loadProject_Click);
            // 
            // Intro
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(271, 135);
            this.Controls.Add(this.loadProject);
            this.Controls.Add(this.newProject);
            this.Name = "Intro";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "What would you like to do?";
            //this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.Intro_FormClosed);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button newProject;
        private System.Windows.Forms.Button loadProject;
    }
}