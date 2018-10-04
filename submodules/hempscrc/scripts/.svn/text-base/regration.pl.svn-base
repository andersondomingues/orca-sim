	#!/usr/bin/perl


		$path =  "/home2/gcastilhos/hemps4.0/branches/distControl/testcases";
		$script = "/home2/gcastilhos/hemps4.0/branches/distControl/scripts/read_log.pl";
		$time = "400";
		$log  = "$path/logs";
		$task = "2";
		
		#$path =~ s/\/$//ig;
		@cs   = (<$path/cenario*>);


	
		chdir("$path/cenario16x16_25");
		system("make all");
		system("/usr/bin/time ./HeMPS.exe -c $time 2>time.time");
	
		chdir("$path/cenario16x16_50");
		system("make all");
		system("/usr/bin/time ./HeMPS.exe -c $time 2>time.time");
	
		chdir("$path/cenario16x16_75");
		system("make all");
		system("/usr/bin/time ./HeMPS.exe -c $time 2>time.time");
	
		chdir("$path/cenario16x16_100");
		system("make all");
		system("/usr/bin/time ./HeMPS.exe -c $time 2>time.time");
