	#!/usr/bin/perl


	if ($ARGV[0] ne "") {

		$path = shift;     #Caminho da aplicacao       
		$iteracoes= shift; #iteracoes        
		
	}

	else {
	  print "ERRO, falta de argumento\n";
	  exit;
	}

	$path =~ s/\/$//ig;
	($aplicacao) = $path;
	$aplicacao =~ s/.*\///ig;
	$aplicacao =~ s/\n//ig;

	@cs   = (<$path/*.c>);

	$size_c = @cs;

	for($i=0;$i<$size_c;$i++){
		
		($cs_old[$i])=$cs[$i];
		$cs_old[$i] =~ s/.*\///ig;
		$cs_old[$i] =~ s/\.c//ig;
		
		($cs_new[$i])=$cs_old[$i];
		$cs_new[$i] = $cs_old[$i]._.$i;

	}
		
	if($aplicacao =~ $path){
		$path="";
	}
	else{
		$path="$path/";

		}
		
	open( C_FILE2, ">$aplicacao\_config.txt" );	
	
	for($j=0;$j<$iteracoes;$j++){
		($aux2)=$path.$aplicacao._.$j;
		$aux=`mkdir $aux2`;
		($aux3)=$aplicacao._.$j;
		
		print C_FILE2 "#$aux3\n";	
						
		foreach $c_file (@cs) {
			($aux_c) = $c_file;
			$aux_c =~ s/.*\///ig;
			($task)=$aux_c;
			$aux_c =~ s/\.c//ig;
			$new_task=$aux_c._.$j;
			
			$aux=`cp $c_file $aux2/$new_task.c`;
			$path_new_task = "$aux2/$new_task.c";
			
			print C_FILE2 "$new_task.c\n";
				
			open( C_FILE, "<$c_file" );
			my @c_lines = <C_FILE>;
			close(C_FILE);


				
			open( C_FILE, ">$path_new_task" );
			foreach $c_line (@c_lines) {
					
				for($i=0;$i<$size_c;$i++){
					if($c_line =~ Send or $c_line =~ Receive){
						$c_line =~ s/,.*$cs_old[$i]/,$cs_old[$i]_$j/ig;
					}
				}
					if($c_line =~ Echo){
						$c_line =~ s/$aplicacao/communication$j/ig;
					}
				

				print C_FILE "$c_line";
			}
			close(C_FILE);
		}

	}
	close(C_FILE2);
