#include <stdio.h>
#include <stdlib.h>
#include<time.h>
#include <string.h>
#include<math.h>

float energy;
float time_on;
//move forward or backward in the field
int move_direction = 0;
//Define energy in Joules
float energy_consumed_for_tracking_event = 0;
float total_energy_battery_per_node = 0;
float energy_consumed_per_switching_event = 0;
float min_energy_required_for_active_node = 0;
float energy_consumed_for_Transmission = 0.0;
float energy_consumed_for_Receiving = 0.0;

//double energy_calculated = 0.0;

//dimensions of the field default=10*10
int dimensions_for_field=10;
int  count = 0;
int dead_nodes = 0;
FILE *Failures, *Targettrajectory, *EnergyDistribution;

/*
Structure: Node
Structure Members: x,y --> define the coordinates of the node
rank --> Unique Id of each node in the field 
time --> holds the value for duty cycle of each node in the field
energy  --> the value of energy remaining in the battery for the sensor node, serves as a parameter for observing the lifetime of the node
status  --> Node active or dead(based on lifetime == 0  for dead node == 1 for live node)
Mode --> Sleep or Awake(== 0 for sleep and == 1 for Awake)
*/
typedef struct
{
	int x, y, rank, time;
	double energy;
	int status;
	int mode;

}node;

node** field;
node *n;

struct target
{
	int x, y, dx, dy, counter;

};

struct target enter(struct target t, int dimensions_for_field)
{
	
	int myArray[2] = {0, dimensions_for_field-1};
    int randomIndex = rand()%2;
    t.x = myArray[randomIndex];
	// move towards t.x = dimensions_for_field-1
	if (t.x == 0)
		move_direction = 0;
	// move backwards in the field i.e towards other boundary t.x =0
	else
		move_direction = 1;
	t.y = rand() % dimensions_for_field;
	fprintf(Targettrajectory, ",%d,%d\n", t.x, t.y);
	return t;
}

struct target move(struct target t, int dimensions_for_field)
{
	int MovementForY[3] = { -1,0,1 };
	int myArray[2];
	if (move_direction == 0)
	{
		myArray[0] = 0;
		myArray[1] = 1;

	}

	else
	{
		myArray[0] = -1;
		myArray[1] = 0;

	}

	int randomIndexforx = rand() % 2;
	t.dx = myArray[randomIndexforx];
	t.x += t.dx;
	int randomIndexfory = rand() % 3;
	t.dy = MovementForY[randomIndexfory];
	t.y += t.dy;
		
	if (t.x > dimensions_for_field - 1)
	{
		t.x -= t.x % (dimensions_for_field - 1);
		move_direction = 1;
	}
	else if (t.x < 0)
	{
		t.x = abs(t.x);
		move_direction = 0;
	}
	if (t.y > dimensions_for_field - 1)
		t.y -= t.y%(dimensions_for_field-1);

	else if (t.y < 0)
		t.y = abs(t.y);

	fprintf(Targettrajectory, ",%d,%d\n", t.x, t.y);
	return t;
}

//Consume Energy for switching function

void consume_energy_while_switching(node *n)
{
	double diffEnergy = 0.0;
	if ((*n).mode == 0)
	{
		diffEnergy = (*n).energy - energy_consumed_per_switching_event;
		if ((check_for_dead_status(diffEnergy, n)))
		{
			(*n).energy = diffEnergy;
			(*n).mode = 1;
		}
			
	}
}

//returns 0 if dead and 1 if alive

int check_for_dead_status(double energy_calculated, node *n)
{
	int ret;
	if (energy_calculated < min_energy_required_for_active_node)
	{
		if ((*n).status == 1)
		{

			fprintf(Failures, ",[%d %d],%lf\n", (*n).x, (*n).y, (*n).energy);
			(*n).status = 0;
		}
		ret=0;
	}
	else
		ret = 1;
	return ret;
}

void active (struct target t, int dimensions_for_field, node** field)
{ 
	fprintf(EnergyDistribution, ",,Node,Energy Scaling Factor,Energy Remaining\n");
	int i, j, count = 0;
	double d = 0;
	double diffEnergy = 0.0;
	double root2 = sqrt(2);
	for (i = 0; i < dimensions_for_field; ++i)
	{
		for (j = 0; j < dimensions_for_field; ++j)
		{	
			if (field[i][j].status == 0)
				break;
			
			d = sqrt((field[i][j].x - t.x)*(field[i][j].x - t.x) + (field[i][j].y - t.y)*(field[i][j].y - t.y));
			if (field[i][j].x == t.x && field[i][j].y == t.y)
			{
				consume_energy_while_switching(&field[i][j]);
				diffEnergy = field[i][j].energy - energy_consumed_for_tracking_event - energy_consumed_for_Transmission;
				if ((check_for_dead_status(diffEnergy,&field[i][j])))
				{
					field[i][j].energy = diffEnergy;
					fprintf(EnergyDistribution, ",,[%d %d],1,%lf\n", field[i][j].x, field[i][j].y, field[i][j].energy);
				}
				
			}

			else if (d == 1.0 || d == root2)
			{
				consume_energy_while_switching(&field[i][j]);
				diffEnergy = field[i][j].energy - (energy_consumed_for_tracking_event*0.5) - energy_consumed_for_Receiving;
				if ((check_for_dead_status(diffEnergy,&field[i][j])))
				{
					field[i][j].energy = diffEnergy;
					fprintf(EnergyDistribution, ",,[%d %d],0.5,%lf\n", field[i][j].x, field[i][j].y, field[i][j].energy);
				}
				
			}

			else if (d == 2.0 || d == (2 * root2) || d == (sqrt(5)))
			{
				consume_energy_while_switching(&field[i][j]);
				diffEnergy = field[i][j].energy - (energy_consumed_for_tracking_event * 0.25) - energy_consumed_for_Receiving;
				if ((check_for_dead_status(diffEnergy,&field[i][j])))
				{
					field[i][j].energy = diffEnergy;
					fprintf(EnergyDistribution, ",,[%d %d],0.25,%lf\n", field[i][j].x, field[i][j].y, field[i][j].energy);
				}
				
			}

			else if (d == 3.0 || d == (3 * root2) || d == (sqrt(10)) || d == (sqrt(13)) || d == (sqrt(18)))
			{
				consume_energy_while_switching(&field[i][j]);
				diffEnergy = field[i][j].energy - (energy_consumed_for_tracking_event * 0.125) - energy_consumed_for_Receiving;
				if ((check_for_dead_status(diffEnergy,&field[i][j])))
				{
					field[i][j].energy = diffEnergy;
					fprintf(EnergyDistribution, ",,[%d %d],0.125,%lf\n", field[i][j].x, field[i][j].y, field[i][j].energy);
				}
				
			}

			
			else if ((field[i][j].x == (t.x + 3) || field[i][j].x == (t.x - 3)) && (field[i][j].y == (t.y + 3) || field[i][j].x == (t.y - 3)))
			{
				consume_energy_while_switching(&field[i][j]);
				diffEnergy = field[i][j].energy - (energy_consumed_for_tracking_event * 0.125) - energy_consumed_for_Receiving;
				if ((check_for_dead_status(diffEnergy,&field[i][j])))
				{
					field[i][j].energy = diffEnergy;
					fprintf(EnergyDistribution, ",,[%d %d],0.125,%lf\n", field[i][j].x, field[i][j].y, field[i][j].energy);
				}
				
			}
			
			else
				field[i][j].mode = 0;

		}
	}

}

int status_of_tracking_network(node **field)
{
	int i, j;
	for (i = 0; i < dimensions_for_field; ++i)
	{
		for (j = 0; j < dimensions_for_field; ++j)
		{
			if (field[i][j].status == 0)
					dead_nodes++;
				
			}
		}
	if ((dead_nodes/count) > 0.5)
		return 0;
	else
		return 1;
}

int main(int argc, char *argv[])
{
FILE *ConfigFile;
char *filename= argv[1];
ConfigFile = fopen(filename, "r");
char *token;


Failures = fopen("Failures_EST.csv", "w+");
Targettrajectory = fopen("Targettrajectory_EST.csv", "w+");
EnergyDistribution = fopen("EnergyDistribution_EST.csv", "w+");

// test for files not existing. 
if (ConfigFile == NULL)
{
	printf("Error! Could not open file\n");
	exit(-1); 
}
char line[200];
while (fgets(line, sizeof line, ConfigFile) != NULL)
{
	token = strtok(line, "=");
	if (strstr(token, "energy_consumed_for_tracking_event"))
	{
		token = strtok(NULL, "=");
		energy_consumed_for_tracking_event = atof(token);
	}
	else if (strstr(token, "total_energy_battery_per_node"))
	{
		token = strtok(NULL, "=");
		total_energy_battery_per_node = atof(token);
	}
	else if (strstr(token, "dimensions_for_field"))
	{
		token = strtok(NULL, "=");
		dimensions_for_field = atoi(token);
	}
	else if (strstr(token, "energy_consumed_per_switching_event"))
	{
		token = strtok(NULL, "=");
		energy_consumed_per_switching_event = atof(token);
	}
	else if (strstr(token, "min_energy_required_for_active_node"))
	{
		token = strtok(NULL, "=");
		min_energy_required_for_active_node = atof(token);
	}
	else if (strstr(token, "energy_consumed_for_Transmission"))
	{
		token = strtok(NULL, "=");
		energy_consumed_for_Transmission = atof(token);
	}
	else if (strstr(token, "energy_consumed_for_Receiving"))
	{
		token = strtok(NULL, "=");
		energy_consumed_for_Receiving = atof(token);
	}
}
fclose(ConfigFile);
printf("Configurations Given by the User\n");
printf("energy_consumed_for_tracking_event %lf\n", energy_consumed_for_tracking_event);
printf("total_energy_battery_per_node %lf\n", total_energy_battery_per_node);
printf("energy_consumed_per_switching_event %lf\n", energy_consumed_per_switching_event);
printf("min_energy_required_for_active_node %lf\n", min_energy_required_for_active_node);
printf("energy_consumed_for_Transmission %lf\n", energy_consumed_for_Transmission);
printf("energy_consumed_for_Receiving %lf\n", energy_consumed_for_Receiving);

printf("dimensions_for_field %d\n", dimensions_for_field);
count = dimensions_for_field * dimensions_for_field;
int i, j, num=0;
node **field = (node**)malloc(dimensions_for_field*sizeof(node *));
for(i=0;i<dimensions_for_field; i++)
field[i]=(node*)malloc(dimensions_for_field*sizeof(node));

	for (i = 0; i < dimensions_for_field; ++i)
	{
		for (j = 0; j < dimensions_for_field; ++j)
		{	
			field[i][j].x = i;
			field[i][j].y = j;
			field[i][j].rank = num++;
			field[i][j].status = 1;
			field[i][j].energy = total_energy_battery_per_node;
			
			if (i == 0 || i == (dimensions_for_field-1) || i == 1 || i == (dimensions_for_field - 2) || i == 2 || i == (dimensions_for_field - 3)\
				|| j == 0 || j == (dimensions_for_field-1) || j == 1 || j == (dimensions_for_field - 2)|| j == 2 || j == (dimensions_for_field - 3))
				field[i][j].mode = 1;
			
			else 
				field[i][j].mode = 0;
		}
	}

	srand(time(0));
	struct target t1;
	fprintf(Targettrajectory, "Target,x-coordinate, y-coordinate\n");
	fprintf(Failures, "Node,[x-coordinate y-coordinate],Energy Remaining\n");
	t1 = enter(t1, dimensions_for_field);
	fprintf(EnergyDistribution, "Target,[%d %d]\n\n", t1.x, t1.y);
	active(t1, dimensions_for_field, field);
	for(i=0; i<1000; i++)
	{
		t1 = move(t1, dimensions_for_field);
		fprintf(EnergyDistribution, "Target,[%d %d]\n\n", t1.x,t1.y);
		if (status_of_tracking_network(field))
		{	
			active(t1, dimensions_for_field, field);
		}

		else
		{
			fprintf(Failures, "Network Died in the loop %d\n", i);
			break;
		}
	}
	
	
}


