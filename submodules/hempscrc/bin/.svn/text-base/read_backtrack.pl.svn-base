#!/usr/bin/perl

#use constant value => "FFFC000FAA8000AA";
use Switch;

#$myString = 'AAAA95555FFFFE00';
#NNNNNNNNNWWWWWWWWWSSSSSSSSSNEEEE

#$myString = 'AAAA97FFFF5AAAA955FFFFE0';
#ENNNNNNNNWWSSSSSSSSSWWNNNNNNNNNWWWWWSSSSSSSSSNEE

#$myString = 'AA65FFD6AAAA5500';
#NNNNNWWWSSSSSWWNNNNNNNNNWWWWEEEE

#$myString = '007FF500';
#EEEEWSSSSSWWEEEE

#$myString = '002A5500';
#EEEEENNNWWWWEEEE

foreach $i (0 .. $#ARGV) {
    $myString = $ARGV[$i];
    my @char_array = split(//,"$myString");
    foreach my $char (@char_array){
        print $char;
    }
    print "\n";
    foreach my $char (@char_array){
        #print $char;
        #print "\n";
        switch ($char){
            case "0"        { print "EE"}
            case "1"        { print "EW"}
            case '2'        { print "EN"}
            case '3'        { print "ES"}
            case '4'        { print "WE"}
            case '5'        { print "WW"}
            case '6'        { print "WN"}
            case '7'        { print "WS"}
            case '8'        { print "NE"}
            case '9'        { print "NW"}
            case ["a","A"]  { print "NN"}
            case ["b","B"]  { print "NS"}
            case ["c","C"]  { print "SE"}
            case ["d","D"]  { print "SW"}
            case ["e","E"]  { print "SN"}
            case ["f","F"]  { print "SS"}
            case default    { print "--"}
        }
        #print "\n";
    }
    print "\n";
    $i=$i+1;
}
