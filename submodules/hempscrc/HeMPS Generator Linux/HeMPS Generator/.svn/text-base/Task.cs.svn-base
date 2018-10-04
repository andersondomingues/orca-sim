using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace HeMPS_Generator {
    public class Task : ListViewItem {
        
        private int priority;
        private int timeSlice;
        private int processor;  /* Task is location (-1: Master) */
        private int coretype;   /* 0 = plasma, 1 = mblite */
        private TreeNode application;

        public Task(string name, int address) {
            Text = name;
            processor = address;
            
            /* Default values */
            coretype = 0;
            priority = 100;
            timeSlice = 16384;            
        }

        public int CoreType{

            get { return coretype; }

            set { coretype = value; }
        }

        public int Priority {

            get { return priority; }

            set { priority = value; }
        }

        public int TimeSlice {

            get { return timeSlice; }

            set { timeSlice = value; }
        }

        public int Processor {

            get { return processor; }

            set { processor = value; }
        }

        public TreeNode Application
        {

            get { return application; }

            set { application = value; }
        }
    }
}
