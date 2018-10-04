namespace HeMPS_Generator
{
    partial class Repository
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
            this.gbxRepository = new System.Windows.Forms.GroupBox();
            this.lvTasks = new System.Windows.Forms.ListView();
            this.gbxRepository.SuspendLayout();
            this.SuspendLayout();
            // 
            // gbxRepository
            // 
            this.gbxRepository.Controls.Add(this.lvTasks);
            this.gbxRepository.Location = new System.Drawing.Point(3, 3);
            this.gbxRepository.Name = "gbxRepository";
            this.gbxRepository.Size = new System.Drawing.Size(120, 206);
            this.gbxRepository.TabIndex = 0;
            this.gbxRepository.TabStop = false;
            this.gbxRepository.Text = "repository";
            // 
            // lvTasks
            // 
            this.lvTasks.AllowDrop = true;
            this.lvTasks.Location = new System.Drawing.Point(6, 31);
            this.lvTasks.Name = "lvTasks";
            this.lvTasks.Size = new System.Drawing.Size(105, 169);
            this.lvTasks.TabIndex = 0;
            this.lvTasks.UseCompatibleStateImageBehavior = false;
            this.lvTasks.View = System.Windows.Forms.View.List;
            this.lvTasks.DragDrop += new System.Windows.Forms.DragEventHandler(this.lvTasks_DragDrop);
            this.lvTasks.DragEnter += new System.Windows.Forms.DragEventHandler(this.lvTasks_DragEnter);
            this.lvTasks.KeyDown += new System.Windows.Forms.KeyEventHandler(this.Delete_KeyDown);
            this.lvTasks.ItemDrag += new System.Windows.Forms.ItemDragEventHandler(this.lvTasks_ItemDrag);
            // 
            // Repository
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.gbxRepository);
            this.Name = "Repository";
            this.Size = new System.Drawing.Size(130, 216);
            this.gbxRepository.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.GroupBox gbxRepository;
        private System.Windows.Forms.ListView lvTasks;

    }
}
