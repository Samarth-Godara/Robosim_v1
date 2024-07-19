
/*
	Robosim V.1 is a robotic simulator, designed to simulate algorithms
	for indoor robots using artificial landmarks on the floor.
	The walls of floor are represented as '#'
	Landmarks are represented as '.'
	And the robot is represented as " ^ , v , < , > " depending on
	the direction it is facing in.

	Working :
	The first step is to input the map-filename
	The simulator calculates the dimensions of the map from file
	Next step is to copy the map into memory
	The simulator then asks the user to initialize the position of robot
	on the floor
	After the initialization the simulation of the algorithm is continued
	The user can change the algorithm as per his requirement.
	The Simulator in the end gives the time elapsed in execution.

	Developed By : Samarth Godara 24-04-2017
*/

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <malloc.h>
#include <dos.h>

//direction definition
#define north 0
#define south 1
#define east 2
#define west 3

//map point description
#define straight 0
#define no_path 1
#define left_turn 2
#define right_turn 3
#define junction 4

#define right 1
#define left 0

typedef struct node		//junction node
{
	int stat[3];            //array to store station number
	int visit;              //keeps track of the visits while scanning
	int no;                 //junction number
	struct node* junct[3];  //array to store the adjacent junction nodes
}j_node;

void get_map_dim();		//calculate the dimension of the map from the file
void copy_map();                //copy the map from file to memory
void print_map();               //print the map on screen
void initialize_robot();        //initialize the position of the robot on the map
int sense_front();              //sense the surrounding for turns, juncitions, etc.
void step_forward();            //take a step forward in the current direction
int check_right_side();         //check if there is path on the right side of robot
int check_left_side();          //check if there is path on the left side of the robot
void print_floor();             //print the floor with the position of robot
void turn_right();              //turn the robot to right
void turn_left();               //turn the robot to left
void turn_around();             //turn around the robot
void choose_right();            //choose the right path at the juncition
void choose_left();             //choose the left path at the junction
int count_1s();			//count the number of '1's around the robot
void print_message();		//prints the information about the simulator
void left_navi(); 		//navigates the whole floor choosing left path on each junction
void right_navi();              //navigate the whole floor choosing right path on each junction
void free_memory();		//deallocate the memory for the map and other runtime memory units

void ai_navigator();            //artificial intelligence navigator algorithm
j_node* scan_path();            //scan the path and make a corresponding map
void push(j_node*);             //stack function
j_node* pop();
void print_stack();
					//extract the junctions to be crossed over to reach destination
int extract_junct(j_node *parent, j_node* head, short station);
int search_station(j_node*, int);	//search station number in a junction node
j_node* get_turns(int s, int d);	//get the heading information at every junction
int s_index(j_node *j, int x);  	//obtain the index of a station in a node
int j_index(j_node *j, j_node* x);      //obtain the index of a node
void execute_traversal();               //traverse the robot
void print_queue_moves();               //print the moves present in stack


//map_x = breadth of the map, map_y = height of the map
//map = memory block for map storage, dir = direction of the robot
//robot_x, robot_y = coordinates of the robot
//filename = filename of the map file, fptr = pointer to read file data
int map_x=0,map_y=0, *map, robot_x, robot_y, dir;
char filename[50];
FILE *fptr;
j_node* stack[100];           		//stack for junction nodes
int tos=-1;
int queue[100],front=-1;                //queue to store moves
int speed=0,rtime=0;


int main()
{
	int ch=0;
	print_message();
	get_map_dim();            //get the file name and read the dimensions
	map = (int *)malloc(sizeof(int)*map_x*map_y);//create memory for map
	copy_map();               //copy the map into memory
	//print_map();            //print the map
	initialize_robot();       //initialize the position of robot on the map
	printf("\nEnter the Speed of the Robot (m/sec) :");
	scanf("%d",&speed);
	printf("\nChoose the Algorithm to execute :\n1: Left Path Algorithm\n2: Right Path Algorithm\n3: A.I. based Localization and Navigational Algorithm");
	fflush(stdin);
	scanf("%d",&ch);
	switch(ch)
	{
		case 1:	left_navi(); 		  //navigate the floor using left path of each junction
			break;
		case 2: right_navi();             //navigate the floor using right path of each junction
			break;
		case 3: ai_navigator();
			break;
		default: printf("Invalid Option...");
	}
	printf("\nTime consumed : %d sec.",rtime);
	free_memory();
	return 0;
}

void clear_stack()
{
	tos=-1;
}

void ai_navigator()			//the AI navigation of the robot
{
	char c=' ';
	int s=1,d;
	j_node *head=NULL;
	head = scan_path();             //scan the path algorithm
	printf("\nTime consumed : %d sec.",rtime);
	while(1)
	{
		printf("\nPlease enter The Destination Station Number :");
		scanf("%d",&d);
		if(s==d) continue;
		extract_junct(NULL, head, d);	//get the junctions from source to destination
		push(head);
		printf("Junctions in between the Source Station %d and Destination Station %d",s,d);
		print_stack();
		printf("\nPress any key to continue Traversal...");
		getch();
		head=get_turns(s,d);		//get the turns to be used at corresponding junctions
		printf("\nTurns to be made at the corresponding junctions...");
		print_queue_moves();
		printf("\nPress any key to continue Traversal...");
		getch();
		execute_traversal();		//execute the traversal using the extracted turn information
		s=d;
		clear_stack();
		fflush(stdin);
		printf("\nDo you wish to continue? (y/n)");
		scanf("%c",&c);
		if(c=='n')break;
	}
}

void print_queue_moves()	//print all the elements of the queue
{
	int i;
	for(i=0;i<=front;i++)
	{
		if(queue[i]==right) printf("\nRight Turn");
		else printf("\nLeft Turn");
	}
}

void execute_traversal()	//execute the traversal using the heading information in the queue
{
	int i,pos;
	for(i=0;i<=front+1;)
	{
		pos=sense_front();
		switch(pos)
		{
			case straight : step_forward();
					break;
			case junction : if(queue[i]==right)	choose_right();
					else if(queue[i]==left) choose_left();
					i++;
					break;
			case left_turn:	turn_left();
					break;
			case right_turn:turn_right();
					break;
			case no_path:	turn_around();
					i++;
					break;
			default :	return;
		}
		//print the current position of robot on the floor
		print_floor();
		print_queue_moves();
		delay(100);
	}

	front=-1;
}

int stack_size()	//return the size of stack
{
	return (tos+1);
}

int sjs(int s, j_node* j, int d)    //generate move between 2 stations
{
	int x,y;
	x=s_index(j,s);
	y=s_index(j,d);
	if((y-(x+1))%3==0) return right;
	else return left;
}

int sjj(int s, j_node* j, j_node* d)	//generate move between station and a junction
{
	int x,y;
	x=s_index(j,s);
	y=j_index(j,d);
	if((y-(x+1))%3==0) return right;
	else return left;
}

int jjs(j_node* s, j_node* j, int d)	//generate move b/w junction and station
{
	int x,y;
	x=j_index(j,s);
	y=s_index(j,d);
	if((y-(x+1))%3==0) return right;
	else return left;
}

int jjj(j_node* s, j_node* j, j_node* d)	//generate move b/w 2 junctions
{
	int x,y;
	x=j_index(j,s);
	y=j_index(j,d);
	if((y-(x+1))%3==0) return right;
	else return left;
}


int s_index(j_node *j, int x)			//find index of station in the node
{
	int i;
	for(i=0;i<3;i++)
	{
		if(j->stat[i]==x) return i;
	}
	return -1;
}

int j_index(j_node *j, j_node* x) 		//find index of junction in the node
{
	int i;
	for(i=0;i<3;i++)
	{
		if(j->junct[i]==x) return i;
	}
	return -1;
}


j_node* get_turns(int s, int d)		//generate the turns to be used while traversal
{
	j_node *j1, *j2, *j3, *head=NULL;
	int move=0;
	if(stack_size()==1)
	{
		j1=pop();
		move=sjs(s,j1,d);
		queue[++front]=move;
		head=j1;
	}
	else if(stack_size()==2)
	{
		j1=pop();
		j2=pop();
		move=sjj(s,j1,j2);
		queue[++front]=move;
		move=jjs(j1,j2,d);
		queue[++front]=move;
		head=j2;
	}
	else
	{
		j1=pop();
		j2=pop();
		move=sjj(s,j1,j2);
		queue[++front]=move;
		push(j2);
		push(j1);
		while(stack_size()>=3)
		{
			j1=pop();
			j2=pop();
			j3=pop();
			move=jjj(j1,j2,j3);
			queue[++front]=move;
			push(j3);
			push(j2);
		}
		j1=pop();
		j2=pop();
		move=jjs(j1,j2,d);
		queue[++front]=move;
		head=j2;
	}
	return head;
}

void print_stack() 		//print all elements of stack
{
	int i;
	printf("\nStack:");
	for(i=tos;i>=0;i--)
	{
		printf("\nJunction : %d",stack[i]->no);
	}
}

int extract_junct(j_node *parent, j_node* head, short station)	//generate the junction nodes to be crossed over to reach the destination
{
	int i;
	if(head==NULL) return 0;
	else if(search_station(head,station)) return 1;
	else
	{
		for(i=0;i<3;i++)
		{
			if((head->junct[i]!=parent) && (extract_junct(head, head->junct[i],station)))
			{
				push(head->junct[i]);
				return 1;
			}

		}
		return 0;
	}
}

int search_station(j_node* head, int station)   //search if the station number is present in the junction node
{
	int i;
	for(i=0;i<3;i++)
	{
		if(head->stat[i]==station) return 1;
	}
	return 0;
}
void push(j_node* x)		//stack operation
{
	tos++;
	stack[tos]=x;
}

j_node* pop()                   //stack operation
{
	if(tos<0) return NULL;
	j_node* x=stack[tos];
	tos--;
	return x;
}

int stack_empty()              //stack operation
{
	if(tos<0) return 1;
	else return 0;
}

int move_on_line()		//more the robot on the line, return 0 on reaching a station and 1 at a junction
{
	int pos=0;
	while(1)
	{       //sense the surroundings
		pos=sense_front();
		//choose left/right path on every junction - this will make the robot traverse the whole floor
		switch(pos)
		{
			case straight : step_forward();
					break;
			case junction :	return 1;
			case left_turn:	turn_left();
					break;
			case right_turn:turn_right();
					break;
			case no_path:	return 0;
			default :	return 0;
		}
		//print the current position of robot on the floor
		print_floor();
		print_stack();
		delay(100);
	}

}

j_node* scan_path()	     	//scan the line network path
{
	int station=1, backtrack=0, p, junct_no=1;
	j_node *x=NULL, *n=NULL, *head=NULL;
	do
	{
		p=move_on_line();

		if(backtrack)
		{
			backtrack=0;
			x=pop();
			if(x->visit==3)
			{
				backtrack=1;
			}
			else
			{
				push(x);
			}
			choose_right();
		}
		else if(p==1)
		{
			n=(j_node *)malloc(sizeof(j_node));
			n->visit=0;
			n->no=junct_no++;
			if(stack_empty())
			{
				n->stat[n->visit]=station;
				n->visit = (n->visit)+1;
				push(n);
				head=n;
			}
			else
			{
				x=pop();
				n->junct[n->visit]=x;
				x->junct[x->visit]=n;
				n->visit = (n->visit)+1;
				x->visit = (x->visit)+1;
				push(x);
				push(n);
			}
			choose_right();
		}
		else if(p==0)
		{
			station++;
			x=pop();
			x->stat[x->visit]=station;
			x->visit=x->visit+1;
			turn_around();
			backtrack=1;
			push(x);
		}
	}
	while(!stack_empty());
	p=move_on_line();
	turn_around();
	return head;
}
void free_memory()
{
	printf("\nDeallocating the Memory...\n");
	free(map);                //free the map memory
	printf("\nTerminating the Simulation...\n\nPress any key to continue...");
	getch();
}

void print_message()
{
	clrscr();
	printf("\n\n\t******             *\n");
	printf("\t*     *            *\n");
	printf("\t*     *            *               *****           *     *\n");
	printf("\t******     *****   *****   *****   *       *****   * * * *\n");
	printf("\t*  *       *   *   *   *   *   *   *****     *     *  *  *\n");
	printf("\t*    *     *   *   *   *   *   *       *     *     *     *\n");
	printf("\t*      *   *****   *****   *****   *****   *****   *     *\n");

	printf("\n\n\tRobosim V.1 is a robotic simulator, designed to simulate algorithms\n\tfor indoor robots using artificial landmarks on the floor.\n\tThe walls of floor are represented as '#'\n\tLandmarks are represented as '.'\n\tAnd the robot is represented as \" ^ , v , < , > \" depending on\n\tthe direction it is facing in. Press any key to continue...");
	getch();
	printf("\n\n\n\tWorking :\n\tThe first step is to input the map-filename\n\tThe simulator calculates the dimensions of the map from file\n\tNext step is to copy the map into memory\n\tThe simulator then asks the user to initialize the position of robot\n\ton the floor\n\tAfter the initialization the simulation of the algorithm is continued\n\tThe user can change the algorithm as per his requirement.\n\tThe Simulator in the end gives the time elapsed in execution.\n\n\tDeveloped By : Samarth Godara 24-04-2017\n\n");
}

int count_1s()			//count the number of 1s around the robot
{
	int c=0;

	//line above robot
	//same line
	if((robot_y-1)>=0 && (robot_y-1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y-1)*map_x)+robot_x)==1)) c++;
	//before robot
	if((robot_y-1)>=0 && (robot_y-1)<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+((robot_y-1)*map_x)+(robot_x-1))==1)) c++;
	//after robot
	if((robot_y-1)>=0 && (robot_y-1)<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+((robot_y-1)*map_x)+(robot_x+1))==1)) c++;

	//line of robot
	//after robot
	if(robot_y>=0 && robot_y<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+(robot_y*map_x)+(robot_x+1))==1)) c++;
	//before robot
	if(robot_y>=0 && robot_y<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+(robot_y*map_x)+(robot_x-1))==1)) c++;
	//same line
	if(robot_y>=0 && robot_y<map_y && robot_x>=0 && robot_x<map_x && (*(map+(robot_y*map_x)+robot_x)==1)) c++;

	//line below robot
	//same line
	if((robot_y+1)>=0 && (robot_y+1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y+1)*map_x)+robot_x)==1)) c++;
	//before robot
	if((robot_y+1)>=0 && (robot_y+1)<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+((robot_y+1)*map_x)+(robot_x-1))==1)) c++;
	//after robot
	if((robot_y+1)>=0 && (robot_y+1)<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+((robot_y+1)*map_x)+(robot_x+1))==1)) c++;

	return c;
}

int check_left_side()		//check if there is path on left side of the robot
{
	//according to the direction of the robo, 3 coordinates of left side of robot are checked
	switch(dir)
	{
		case north :	if(*(map+((robot_y-1)*map_x)+(robot_x-1))==1 || *(map+((robot_y)*map_x)+(robot_x-1))==1 || *(map+((robot_y+1)*map_x)+(robot_x-1))==1)
				return 1; break;
		case south :    if(*(map+((robot_y-1)*map_x)+(robot_x+1))==1 || *(map+((robot_y)*map_x)+(robot_x+1))==1 || *(map+((robot_y+1)*map_x)+(robot_x+1))==1)
				return 1; break;
		case east :     if(*(map+((robot_y-1)*map_x)+(robot_x-1))==1 || *(map+((robot_y-1)*map_x)+(robot_x))==1 || *(map+((robot_y-1)*map_x)+(robot_x+1))==1)
				return 1; break;
		case west :   	if(*(map+((robot_y+1)*map_x)+(robot_x-1))==1 || *(map+((robot_y+1)*map_x)+(robot_x))==1 || *(map+((robot_y+1)*map_x)+(robot_x+1))==1)
				return 1; break;
	}
	return 0;
}

int check_right_side()          //check if there is path on right side of the robot
{
	//according to the direction of the robo, 3 coordinates of right side of robot are checked
	switch(dir)
	{
		case north :	if(*(map+((robot_y-1)*map_x)+(robot_x+1))==1 || *(map+((robot_y)*map_x)+(robot_x+1))==1 || *(map+((robot_y+1)*map_x)+(robot_x+1))==1)
				return 1;  break;
		case south :    if(*(map+((robot_y-1)*map_x)+(robot_x-1))==1 || *(map+((robot_y)*map_x)+(robot_x-1))==1 || *(map+((robot_y+1)*map_x)+(robot_x-1))==1)
				return 1;  break;
		case east :   	if(*(map+((robot_y+1)*map_x)+(robot_x-1))==1 || *(map+((robot_y+1)*map_x)+(robot_x))==1 || *(map+((robot_y+1)*map_x)+(robot_x+1))==1)
				return 1;  break;
		case west :   	if(*(map+((robot_y-1)*map_x)+(robot_x-1))==1 || *(map+((robot_y-1)*map_x)+(robot_x))==1 || *(map+((robot_y-1)*map_x)+(robot_x+1))==1)
				return 1;  break;
	}
	return 0;
}

int sense_front()
{
	int c=0;
	switch(dir)
	{
		case north:
				//line before robot
				if((robot_y-1)>=0 && (robot_y-1)<map_y && (robot_x)>=0 && (robot_x)<map_x && (*(map+((robot_y-1)*map_x)+(robot_x))==1)) c++;
				//line of the robot
				//after robot
				if(robot_y>=0 && robot_y<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+(robot_y*map_x)+(robot_x+1))==1)) c++;
				//before
				if(robot_y>=0 && robot_y<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+(robot_y*map_x)+(robot_x-1))==1)) c++;
				//middle
				if(robot_y>=0 && robot_y<map_y && robot_x>=0 && robot_x<map_x && (*(map+(robot_y*map_x)+robot_x)==1)) c++;
				//if there is one '1' around, either there is no path or the line is straight
				if(c==1)
				{
					//case for no path in front
					if((robot_y-1)>=0 && (robot_y-1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y-1)*map_x)+robot_x)==0)) return no_path;
					//else its a straight path
					else return straight;
				}
				//if there are three '1's around, its a junciton
				else if(c>=3) return junction;
				//if there are two '1's around, its a turn or a straight path
				else if(c==2)
				{
					//check for the right turn
					if(robot_y>=0 && robot_y<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+(robot_y*map_x)+(robot_x+1))==1)) return right_turn;
					//check for the left turn
					else if(robot_y>=0 && robot_y<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+(robot_y*map_x)+(robot_x-1))==1)) return left_turn;
					//else its a straight
					else return straight;
				}
				break;
		case south:
				if(robot_y>=0 && robot_y<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+(robot_y*map_x)+(robot_x+1))==1)) c++;
				if(robot_y>=0 && robot_y<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+(robot_y*map_x)+(robot_x-1))==1)) c++;
				if(robot_y>=0 && robot_y<map_y && robot_x>=0 && robot_x<map_x && (*(map+(robot_y*map_x)+robot_x)==1)) c++;
				if((robot_y+1)>=0 && (robot_y+1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y+1)*map_x)+robot_x)==1)) c++;
				if(c==1)
				{
					if((robot_y+1)>=0 && (robot_y+1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y+1)*map_x)+robot_x)==0)) return no_path;
					else return straight;
				}
				else if(c>=3) return junction;
				else if(c==2)
				{
					if(robot_y>=0 && robot_y<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+(robot_y*map_x)+(robot_x+1))==1)) return left_turn;
					else if(robot_y>=0 && robot_y<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+(robot_y*map_x)+(robot_x-1))==1)) return right_turn;
					else return straight;
				}
				break;
		case east :
				if((robot_y-1)>=0 && (robot_y-1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y-1)*map_x)+robot_x)==1)) c++;
				if(robot_y>=0 && robot_y<map_y && robot_x>=0 && robot_x<map_x && (*(map+(robot_y*map_x)+robot_x)==1)) c++;
				if(robot_y>=0 && robot_y<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+(robot_y*map_x)+(robot_x+1))==1)) c++;
				if((robot_y+1)>=0 && (robot_y+1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y+1)*map_x)+robot_x)==1)) c++;
				if(c==1)
				{
					if(robot_y>=0 && robot_y<map_y && (robot_x+1)>=0 && (robot_x+1)<map_x && (*(map+(robot_y*map_x)+(robot_x+1))==0)) return no_path;
					else	return straight;
				}
				else if(c>=3) return junction;
				else if(c==2)
				{
					if((robot_y-1)>=0 && (robot_y-1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y-1)*map_x)+robot_x)==1)) return left_turn;
					else if((robot_y+1)>=0 && (robot_y+1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y+1)*map_x)+robot_x)==1)) return right_turn;
					else return straight;
				}
				break;
		case west:
				if((robot_y-1)>=0 && (robot_y-1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y-1)*map_x)+robot_x)==1)) c++;
				if(robot_y>=0 && robot_y<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+(robot_y*map_x)+(robot_x-1))==1)) c++;
				if(robot_y>=0 && robot_y<map_y && robot_x>=0 && robot_x<map_x && (*(map+(robot_y*map_x)+robot_x)==1)) c++;
				if((robot_y+1)>=0 && (robot_y+1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y+1)*map_x)+robot_x)==1)) c++;
				if(c==1)
				{
					if(robot_y>=0 && robot_y<map_y && (robot_x-1)>=0 && (robot_x-1)<map_x && (*(map+(robot_y*map_x)+(robot_x-1))==0)) return no_path;
					else return straight;
				}
				else if(c>=3) return junction;
				else if(c==2)
				{
					if((robot_y-1)>=0 && (robot_y-1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y-1)*map_x)+robot_x)==1)) return right_turn;
					else if((robot_y+1)>=0 && (robot_y+1)<map_y && robot_x>=0 && robot_x<map_x && (*(map+((robot_y+1)*map_x)+robot_x)==1)) return left_turn;
					else return straight;
				}
				break;
	}
	return no_path;
}

void step_forward()		//step the robot forward in its current direction
{
	rtime+=speed;
	switch(dir)
	{
		case north:     if(robot_y>0)
					robot_y--;
				break;
		case south:     if(robot_y<(map_y-1))
					robot_y++;
				break;
		case east:     if(robot_x<(map_x-1))
					robot_x++;
				break;
		case west:     if(robot_x>0)
					robot_x--;
				break;
	}
}

void turn_around()		//turn the robot around
{
	rtime+=speed;
	switch(dir)
	{
		case north :  	dir=south;
				break;
		case south :    dir=north;
				break;
		case east:	dir=west;
				break;
		case west:	dir=east;
				break;
	}
}

void turn_right()		//make the right turn
{
	rtime+=speed;
	switch(dir)
	{
		case north:	robot_x++;
				dir=east;
				break;
		case south:	robot_x--;
				dir=west;
				break;
		case east:	robot_y++;
				dir=south;
				break;
		case west:	robot_y--;
				dir=north;
				break;
	}
}

void choose_right()		//choose the right path on the junction
{
	rtime+=speed;
	switch(dir)
	{
		case north:     //there are 2 types of junction T type and l- type
				//checking for T type
				if(*(map+(robot_y*map_x)+(robot_x+1))==1)
				{
					robot_x++;
					dir=east;
				}
				//checking for  l- type
				else	robot_y--;
				break;
		case south:	if(*(map+(robot_y*map_x)+(robot_x-1))==1)
				{
					robot_x--;
					dir=west;
				}
				else robot_y++;
				break;
		case east:	if(*(map+((robot_y+1)*map_x)+robot_x)==1)
				{
					robot_y++;
					dir=south;
				}
				else robot_x++;
				break;
		case west:	if(*(map+((robot_y-1)*map_x)+robot_x)==1)
				{
					robot_y--;
					dir=north;
				}
				else robot_x--;
				break;
	}
}


void choose_left()
{
	rtime+=speed;
	switch(dir)
	{                       //there are 2 types of junction T type and l- type
				//checking for T type
		case north:	if(*(map+(robot_y*map_x)+(robot_x-1))==1)
				{
					robot_x--;
					dir=west;
				}
				//for I- type
				else	robot_y--;
				break;
		case south:	if(*(map+(robot_y*map_x)+(robot_x+1))==1)
				{
					robot_x++;
					dir=east;
				}
				else robot_y++;
				break;
		case east:	if(*(map+((robot_y-1)*map_x)+robot_x)==1)
				{
					robot_y--;
					dir=north;
				}
				else robot_x++;
				break;
		case west:	if(*(map+((robot_y+1)*map_x)+robot_x)==1)
				{
					robot_y++;
					dir=south;
				}
				else robot_x--;
				break;
	}
}


void turn_left()                //make left turn
{
	rtime+=speed;
	switch(dir)
	{
		case north:	robot_x--;
				dir=west;
				break;
		case south:	robot_x++;
				dir=east;
				break;
		case east:	robot_y--;
				dir=north;
				break;
		case west:	robot_y++;
				dir=south;
				break;
	}
}

void right_navi()		//algorithm to test the simulator functionalities
{
	int pos=0;
	char c=' ';
	while(1)
	{       //sense the surroundings
		pos=sense_front();
		//choose left/right path on every junction - this will make the robot traverse the whole floor
		switch(pos)
		{
			case straight : step_forward();
					break;
			case junction :	choose_right();
					break;
			case left_turn:	turn_left();
					break;
			case right_turn:turn_right();
					break;
			case no_path:	turn_around();
					break;
			default :	return;
		}
		//print the current position of robot on the floor
		print_floor();
		printf("\nPress 'q' to Exit... Any other key to continue simulation...");
		c=getch();
		if(c=='q') break;
	}
}


void left_navi()		//algorithm to test the simulator functionalities
{
	int pos=0;
	char c=' ';
	while(1)
	{       //sense the surroundings
		pos=sense_front();
		//choose left/right path on every junction - this will make the robot traverse the whole floor
		switch(pos)
		{
			case straight : step_forward();
					break;
			case junction :	choose_left();
					break;
			case left_turn:	turn_left();
					break;
			case right_turn:turn_right();
					break;
			case no_path:	turn_around();
					break;
			default :	return;
		}
		//print the current position of robot on the floor
		print_floor();
		printf("\nPress 'q' to Exit... Any other key to continue simulation...");
		c=getch();
		if(c=='q') break;
	}
}

void initialize_robot()		//initialize the position of robot on the floor
{
	int x,y,i,j;
	char c=' ';
	dir=north;              //initial direction = north
	x=y=0;                  //initial coordinates of the robot

	while(1)
	{
		clrscr();
		printf("\n\nInitialize The Robot...\nw - up  s - down  a - left  d - right\n\n");
		//print the map along with the robo
		for(i=0;i<map_y;i++)
		{
			for(j=0;j<map_x;j++)
			{
				printf(" ");
				//if the coordinates represent the robo
				if(i==y && j==x)
				{
					switch(dir)
					{
						//there is different symbols to represent robot for each direction
						case north: 	printf("^");
								break;
						case south: 	printf("v");
								break;
						case east: 	printf(">");
								break;
						case west: 	printf("<");
								break;
					}
				}
				//if the coordinates represent the floor, print the different parts of the map
				else if(*(map+(i*map_x)+j)==2) printf("#");
				else if(*(map+(i*map_x)+j)==1) printf(".");
				else if(*(map+(i*map_x)+j)==0) printf(" ");
			}
			printf("\n");
		}
		//according to the a,w,s,d keys shift thr robot up,down, right, left
		c=getch();
		if(c=='w')
		{
			dir=north;
			if(y>0)y--;
		}
		else if(c=='s')
		{
			dir=south;
			if(y<map_y)y++;
		}
		else if(c=='a')
		{
			dir=west;
			if(x>0)x--;
		}
		else if(c=='d')
		{
			dir=east;
			if(x<map_x)x++;
		}
		//if user press 'enter' terminate the initialization process
		else if(c==13)
		{
			if(*(map+(y*map_x)+x)!=1)
			{
				printf("\nUnable to initialize the robot at the current position. Press any key to retry\nq - Exit");
				c=getch();
				if(c=='q') exit(0);
				continue;
			}
			robot_x=x;
			robot_y=y;
			printf("\nRobot is Initialized at position %d,%d",x,y);
			return;
		}
	}
}

void print_map()	      	//print the map  in the numbers as stored in the file
{
	int i,j;
	printf("\n");
	for(i=0;i<map_y;i++)
	{
		for(j=0;j<map_x;j++)
		{
			//get the data out of the memory block
			printf(" %d",*(map+(i*map_x)+j));
		}
		printf("\n");
	}
}

void print_floor()		//print the floor along with the robot
{
	int i,j;
	clrscr();
	printf("\n");
	for(i=0;i<map_y;i++)
	{
		for(j=0;j<map_x;j++)
		{
			printf(" ");
			//if the coordinates represent the robot
			if(robot_y==i && robot_x==j)
			{
				switch(dir)
				{
					//there is different symbol of robot for every direction
					case north :	printf("^");
							break;
					case south :	printf("v");
							break;
					case east :	printf(">");
							break;
					case west :	printf("<");
							break;
				}
			}
			//if the coordinates represent the floor, print the coresponding symbol
			else if(*(map+(i*map_x)+j)==2)
				printf("#");
			else if(*(map+(i*map_x)+j)==1)
				printf(".");
			else if(*(map+(i*map_x)+j)==0)
				printf(" ");
		}
		printf("\n");
	}
}


void copy_map()			//copy the map from file into the memory
{
	int i=0,j=0;
	char c=' ';
	//clrscr();
	printf("\n\nCopying Mapfile...");
	//open the file in read mode
	fptr = fopen(filename, "r");
	//printf("\n\nOpening file: %s", filename);
	//printf("\n\nPress any key to continue...");
	getch();
	if(fptr == NULL)
	{
		printf("Error!");
		exit(1);
	}

	//read while file is finished
	while(c!=EOF)
	{
		//get single characters from the file
		c=getc(fptr);
		//if the character is '2' save 2 in the corresponding place in the memory
		if(c=='2')
		{
			*(map+(i*map_x)+j)=2;
			j++;
		}
		//else if character is '1' save 1 in memory
		else if(c=='1')
		{
			*(map+(i*map_x)+j)=1;
			j++;
		}
		else if(c=='0')
		{
			*(map+(i*map_x)+j)=0;
			j++;
		}
		//if the character is next-line increment the line number in memory and start from zero
		else if(c=='\n')
		{
			i++;
			j=0;
		}
	}
	//close the file
	fclose(fptr);
	printf("\nDone Copying Map...\nPress any key to continue...");
	getch();
}

void get_map_dim()		//get the map dimensions from the file to allocate a memory block
{
	int i,j,flag=0;
	char c=' ';

	//store the filename
	printf("\nEnter the name of Map File :");
	scanf("%s",filename);
	printf("\nOpening File : %s",filename);
	//open the file in read mode
	fptr = fopen(filename, "r");
	if(fptr == NULL)
	{
		printf("\nError reading file, press any key to exit...");
		getch();
		exit(1);
	}

	printf("\nCalculating Map Dimensions...");
	i=j=flag=0;
	//read the file until its end
	while(c!=EOF)
	{
		//read single character from file
		c=getc(fptr);
		//if there comes a next-line character, stop calculating the columns, and calculate the number of rows
		if(c=='\n')
		{
			flag=1;
			j++;
		}
		//if there comes a character except the next-line, count it for number of colomns
		else
		{
			if(flag==0) i++;
		}
	}
	//store the counted rows and colomns
	map_x=i;
	map_y=++j;
	printf("\nMap Dimensions : %d x %d",map_x,map_y);
	//close the file
	fclose(fptr);
	printf("\nPress any key to continue...");
	getch();
}