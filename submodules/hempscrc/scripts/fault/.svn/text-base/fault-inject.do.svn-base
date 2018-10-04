# this script reads a list of signals where faults must be injected
#

if {[file exists "signals"] == 1} then {
   puts "injecting faults!"
   set fault [open "signals" r]
   set signals [read $fault]
   set lines [split $signals "\n"]
   close $fault

   foreach line $lines {
     #removes arbitrary number of whitespaces
	 set tokens [split [regsub { {2,}} $line " "] " "]
     
     set number [llength $line]
	 if { $number == 4 } { 
       set force_cmd [format "force %s %s %s %s -cancel 1000 ms" [lindex $tokens 0] [lindex $tokens 1] [lindex $tokens 2] [lindex $tokens 3]]
     }
     if { $number ==6 } {
       set force_cmd [format "force %s %s %s %s -cancel %s %s" [lindex $tokens 0] [lindex $tokens 1] [lindex $tokens 2] [lindex $tokens 3] [lindex $tokens 4] [lindex $tokens 5]]
     }
     puts $force_cmd
     eval $force_cmd
   }    
} else {
   puts "no fault injection"
}
