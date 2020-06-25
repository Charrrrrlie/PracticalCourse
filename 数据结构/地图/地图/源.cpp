#include<stdio.h>
#include<iostream>
#include<string.h>
#include<limits>
#include<vector>
#include<string>

#include"Timer.h"

using namespace std;

#define CityNum 199    //城市数量
#define RouteNum 1975   //路径数量

struct City  //（类似于顶点） 城市
{ 
	double longitude, latitude;    //城市的经度 纬度
	char city[30], country[30];    //城市名 城市所在国家
};

struct Route  //（类似于边）  路径 
{
	char origin_city[30], destin_city[30];  //出发城市和目的地城市
	string transport;       //交通工具
	double time, cost;
	string info;          //城市相关信息网站
};

struct Key          //关键字搜索结构
{
	int key_num;
	char city_name[30];
};

struct Arc         //边节点的结构（用于时间和消费两种优先）
{
	double cost;    //所需
	double time;    //所需时间
	string transport;
	string info;
};

struct Graph 
{
	int arcnum;       //边的个数（路径数量）
	int vexnum;       //节点个数（国家的个数）
	struct Arc **arcs;       //邻接矩阵
	bool pass = false;    //Dijkistra 遍历判断条件
};

struct Way           //用于Dijkistra中记录经过的点
{
	vector <int> city;
	vector <double> cost;
	vector <string> info;
	vector <string> transport;
};

double distance_D[CityNum];      //表示Dijkistra距离
bool visit[CityNum] = { false };                    //判断点是否被访问

Way way[CityNum];                           //存储路径
Graph g;
//自编功能函数//
//匹配城市与关键字
int LocateCity(char *city, Key *key,int num)
{
	for (int i = 0; i < num; i++)    //可用KMP扩充
	{
		if (strcmp(key[i].city_name, city) == 0)   //找到城市则返回城市关键字
			return key[i].key_num;
	
	}
}

char *Match(int key_num, Key *key)
{
	for(int i=0;i<CityNum;i++)
		if(key[i].key_num==key_num)
		return key[i].city_name;
}


//最小值
double Min(double x, double y)
{
	return x <= y ?  x : y;
}


//读取城市信息
void ReadCityFile(const char *source,City *city,Key *key)     
{
	FILE *fp = fopen(source, "rt");
	if (!fp)
	{
		printf("Error.CityFile Cannot be open./n");
		return;
	}
	
	char ch;         //逐个读取字符串输入到city数组中
	int i = 0 , j = 0;       //计数器

	while (!feof(fp) && i<CityNum)
	{
		ch = fgetc(fp);
		for (; ch != ','; ch = fgetc(fp))   //读取国家
		{
			city[i].country[j] = ch;
			j++;
		}
		city[i].country[j] = '\0';    //添加终止符

		ch = fgetc(fp);
		j = 0;      //计数器清零

		for (; ch != ','; ch = fgetc(fp))   //读取城市
		{
			city[i].city[j] = ch;
			j++;
		}
		city[i].city[j] = '\0';    //添加终止符

		fscanf(fp, "%lf", &city[i].latitude);
		ch = fgetc(fp);
		fscanf(fp, "%lf", &city[i].longitude);

		strcpy(key[i].city_name, city[i].city);   //给城市关键字一一对应
		key[i].key_num = i;                       //第一个城市编号为0 


		ch = fgetc(fp);    //读取换行符
		i++;     //读取下一个城市
		j = 0;
	}
	
    fclose(fp);
}

//读取路径信息
void ReadRouteFile(const char *source, Route *route)  
{
	FILE *fp = fopen(source, "rt");
	if (!fp)
	{
		printf("Error.RouteFile cannot be open.\n");
		return;
	}

	int i = 0;    //计数器
	char *p;      //指针数组指向读取路线中成员信息
	char ch;          //逐个读取字符串输入到city数组中
	while (!feof(fp) && i < RouteNum)
	{
		ch = fgetc(fp);
		for (p = route[i].origin_city; ch != ','; ch = fgetc(fp))     //读取出发城市
		{
			*(p++) = ch;
		}
		*p = '\0';          //添加终止符

		ch = fgetc(fp);
		for (p = route[i].destin_city; ch != ','; ch = fgetc(fp))     //读取目的地城市
		{
			*(p++) = ch;
		}
		*p = '\0';          //添加终止符

		ch = fgetc(fp);
		for (; ch != ','; ch = fgetc(fp))     //读取交通工具
		{
			route[i].transport += ch;
		}

		
		fscanf(fp, "%lf", &route[i].time);   //读取时间
		ch = fgetc(fp);           //读取逗号
		fscanf(fp, "%lf", &route[i].cost);   //读取价格
		ch = fgetc(fp);           //读取逗号

		ch = fgetc(fp);
		while (ch != '\n')
		{
			route[i].info +=ch;
			ch = getc(fp);
		}

		i++;     //读取下一条路径
	}
    
	fclose(fp);
}

void CreateGraph(City *city, Route *route,Key *key,char *priority){
	g.arcnum = RouteNum;
	g.vexnum = CityNum;
	
	g.arcs = new struct Arc*[CityNum];
	for (int j = 0; j < g.vexnum; j++)
	{
		g.arcs[j] = new struct Arc[CityNum];
	}

	for(int i=0;i<g.vexnum;i++)
		for (int j = 0; j < g.vexnum; j++)
		{
			if (i == j)             //初始化边
			{
				g.arcs[i][j].cost = g.arcs[i][j].time = 0;    //初始化零值
				g.arcs[i][j].transport="No need";
				g.arcs[i][j].info="Nothing";
			}
			else
			{
				g.arcs[i][j].cost = g.arcs[i][j].time = DBL_MAX;    //初始化最大值
			}
		}
	int origin_num, dest_num;       //找到始、终城市对应字符的关键字
	for (int i = 0; i < RouteNum; i++)
	{
		

		origin_num = LocateCity(route[i].origin_city, key, g.vexnum);
		dest_num = LocateCity(route[i].destin_city, key, g.vexnum);

		if (strcmp(priority, "TIME") == 0)   //如果价格优先
		{
			if (g.arcs[origin_num][dest_num].cost != DBL_MAX          //只用判断cost在前一次被传入即可
				&& route[i].time< g.arcs[origin_num][dest_num].time)   //对于两城市之间有多种选择
			   {
				g.arcs[origin_num][dest_num].cost = route[i].cost;
				g.arcs[origin_num][dest_num].time = route[i].time;
				g.arcs[origin_num][dest_num].transport = route[i].transport;
				g.arcs[origin_num][dest_num].info = route[i].info;
			   }			
				
			else if(g.arcs[origin_num][dest_num].cost = DBL_MAX){
				g.arcs[origin_num][dest_num].cost = route[i].cost;
				g.arcs[origin_num][dest_num].time = route[i].time;
				g.arcs[origin_num][dest_num].transport = route[i].transport;
				g.arcs[origin_num][dest_num].info = route[i].info;
			}
		}
		else
		{
			if (g.arcs[origin_num][dest_num].cost != DBL_MAX  
				&& route[i].cost < g.arcs[origin_num][dest_num].cost)    
			{
				g.arcs[origin_num][dest_num].cost = route[i].cost;
				g.arcs[origin_num][dest_num].time = route[i].time;
				g.arcs[origin_num][dest_num].transport = route[i].transport;
				g.arcs[origin_num][dest_num].info = route[i].info;
			}
			else if (g.arcs[origin_num][dest_num].cost = DBL_MAX) {
				g.arcs[origin_num][dest_num].cost = route[i].cost;
				g.arcs[origin_num][dest_num].time = route[i].time;
				g.arcs[origin_num][dest_num].transport = route[i].transport;
				g.arcs[origin_num][dest_num].info = route[i].info;
			}
		}
	 }

}


void Dijkistra(Graph &graph, int depart, int destination, char *priority)
{
	int start = depart;    //标记搜索开始点

	for (int i = 0; i < graph.vexnum; i++)
	{
		distance_D[i] = DBL_MAX;
	}

	for (int i = 0; i < graph.vexnum; i++)
	{
		way[i].city.push_back(depart);
		way[i].info.push_back(graph.arcs[depart][i].info);
		way[i].transport.push_back(graph.arcs[depart][i].transport);
		if (strcmp(priority, "TIME") == 0)
			way[i].cost.push_back(graph.arcs[depart][i].time);
		else if (strcmp(priority, "PRICE") == 0)
			way[i].cost.push_back(graph.arcs[depart][i].cost);
	}

	//时间优先
	if (strcmp(priority, "TIME") == 0)     //判断时间优先还是距离优先
	{
		for (int i = 0; i < graph.vexnum; i++)          //导入图价格信息 更新距离数组
		{
			distance_D[i] = Min(distance_D[i], graph.arcs[start][i].time);
		}

		//Dijkistra 遍历
		for (int i = 0; i < graph.vexnum; i++)
		{
			double min = DBL_MAX;          //记录最短路的路径长
			for (int j = 0; j < graph.vexnum; j++)
				if (visit[j] == false && distance_D[j] < min)
				{
					min = distance_D[j];
					start = j;            //记录最短距离的值 继续搜索
				}
			visit[start] = true;              //该点已经遍历


			for (int i = 0; i < graph.vexnum; i++)
			{
				if (visit[i] == false && distance_D[i] > distance_D[start] + graph.arcs[start][i].time)
				{

					way[i].city = way[start].city;
					way[i].city.push_back(start);

					way[i].cost = way[start].cost;
					way[i].cost.push_back(graph.arcs[start][i].time);

					way[i].info = way[start].info;
					way[i].info.push_back(graph.arcs[start][i].info);

					way[i].transport = way[start].transport;
					way[i].transport.push_back(graph.arcs[start][i].transport);

				}
				//记录各点通过路径

				if (visit[i] == false) {
					distance_D[i] = Min(distance_D[i],
						distance_D[start] + graph.arcs[start][i].time);
				}
			}

		}
		way[destination].city.push_back(destination);
	}


	//价格优先
	else if (strcmp(priority, "PRICE") == 0)
	{
		for (int i = 0; i < graph.vexnum; i++)          //导入图价格信息 更新距离数组
		{
			distance_D[i] = Min(distance_D[i], graph.arcs[start][i].cost);
		}

		//Dijkistra 遍历
		for (int i = 0; i < graph.vexnum; i++)
		{
			double min = DBL_MAX;          //记录最短路的路径长
			for (int j = 0; j < graph.vexnum; j++)
				if (visit[j] == false && distance_D[j] < min)
				{
					min = distance_D[j];
					start = j;            //记录最短距离的值 继续搜索
				}
			visit[start] = true;              //该点已经遍历


			for (int i = 0; i < graph.vexnum; i++)
			{
				if (visit[i] == false && distance_D[i] > distance_D[start] + graph.arcs[start][i].cost)
				{

					way[i].city = way[start].city;
					way[i].city.push_back(start);

					way[i].cost = way[start].cost;
					way[i].cost.push_back(graph.arcs[start][i].cost);

					way[i].info = way[start].info;
					way[i].info.push_back(graph.arcs[start][i].info);

					way[i].transport = way[start].transport;
					way[i].transport.push_back(graph.arcs[start][i].transport);

				}
				//记录各点通过路径

				if (visit[i] == false) {
					distance_D[i] = Min(distance_D[i],
						distance_D[start] + graph.arcs[start][i].cost);
				}
			}

		}
		way[destination].city.push_back(destination);
	}
}

//屏幕显示
void Display(int destination_num,Key *key,char *priority)
{
	int i =1;
	printf("途径城市为：");
	for (int j=0;j<way[destination_num].city.size();j++)
	{
		
		char *city = Match(way[destination_num].city[j], key);
		printf("%s ", city);
		if ( i<way[destination_num].city.size())
		{
			printf("---> ");
			i++;
		}
	}

	i = 1;        //初始化
	if(strcmp(priority,"TIME")==0)  printf("\n时间为:");
	else if (strcmp(priority, "PRICE") == 0) printf("\n花费为:");
	for (int j = 0; j < way[destination_num].cost.size(); j++)
	{
		cout << way[destination_num].cost[j];
		if (i < way[destination_num].cost.size())
		{
			printf("---> ");
			i++;
		}
	}

	i = 1;        //初始化
	printf("\n交通工具为:");
	for (int j = 0;j < way[destination_num].transport.size();j++)
	{
		cout << way[destination_num].transport[j];
		if (i < way[destination_num].transport.size())
		{
			printf("---> ");
			i++;
		}
	}
	i = 1;
	printf("\n路径信息为:");
	for (int j = 0; j < way[destination_num].info.size(); j++)
	{
		cout << way[destination_num].info[j]<<endl;
		if (i < way[destination_num].info.size())
		{
			printf("---> ");
			i++;
		}
	}



 }


void DFS(Route *route, int v, Key *key)
{
	visit[v] = true;
	char *city = Match(v,key);
	printf("%s\n", city);        //深度优先遍历输出
	int origin_num, dest_num;
	int i = 0;
	for (; i < RouteNum; i++)        //如果找到一个路线的起始城市关键值和要遍历的相同
		                            //则遍历该路线的目的地城市
	{
		if (!visit[v] && v == LocateCity(route[i].origin_city, key, CityNum))
		{
			origin_num = LocateCity(route[i].origin_city, key, g.vexnum);
			dest_num = LocateCity(route[i].destin_city, key, g.vexnum);
			DFS(route, dest_num, key);
			break;
		}
	}
}

void DFSTraverse(Route *route,Key *key)   //深度
{
	for (int i = 0; i < g.vexnum; i++)
	{
		visit[i] = false;
	}
	for (int i = 0; i < g.vexnum; i++)
	{
		if (!visit[i]) DFS(route,i,key);    //对未访问的顶点调用DFS
	}

}

//地图显示
void VisualDisplay(City *city,Route *route,char *departure, char *destination, const char* address_file_save,Key *key)
{
	FILE *fp = fopen(address_file_save, "wt");
	if (!fp)
	{
		printf("Error.File for VisualDisplay cannot be open.");
	    return;
     }
	int destination_num = LocateCity(destination, key, CityNum);
	int departure_num = LocateCity(departure, key, CityNum);

	for (int i=0;i+1< way[destination_num].city.size();i++)    //判断对于N'Djamena 引起歧义的城市
	{
		char change_city[30];
		strcpy(change_city, Match(way[destination_num].city[i], key));
		for (int j = 0; j < strlen(change_city); j++)
		{
			if (change_city[j] == '\'')     //含有’ 添加转义符后移动数组
			{
				char ch,trans;
				ch = change_city[j];
				change_city[j] = '\\';
				for (int x = j+1; x <= strlen(change_city) - j+1; x++)
				{
					trans= change_city[x];
					change_city[x] = ch;
					ch = trans;
				}
				strcpy(city[way[destination_num].city[i]].city, change_city);
			}
			
		}
	}


	//初始化部分存储
	fprintf(fp, "<!DOCTYPE html><html><head><style type='text/css'>");   //%%才能输入%
	fprintf(fp, "body, html{width: 100%%;height: 100%%;margin:0;font-family:'微软雅黑';}");
	fprintf(fp, "#allmap{height:100%%;width:100%%;}#r-result{width:100%%;}");
	fprintf(fp, "</style><script type = 'text/javascript' src ='");
	fprintf(fp, "http://api.map.baidu.com/api?v=2.0&ak=nSxiPohfziUaCuONe4ViUP2N'>");
	fprintf(fp, "</script><title>Shortest path from %s to %s",departure,destination);   //表头的城市
	fprintf(fp, "</title></head><body><div id='allmap'>");
	fprintf(fp, "</div></div></body></html><script type='text/javascript'>");
	fprintf(fp, "var map = new BMap.Map('allmap');");
	fprintf(fp, "var point = new BMap.Point(0,0);map.centerAndZoom(point,2);");
	fprintf(fp, "map.enableScrollWheelZoom(true);var marker0 = new BMap.Marker(new BMap.Point(%lf,%lf));"
	                     , city[departure_num].longitude,city[departure_num].latitude);
	fprintf(fp, "map.addOverlay(marker0);\n");

	//存储路径及信息
	//出发点单独存（没有路径等信息)
	fprintf(fp, "var infoWindow0 = new BMap.InfoWindow");
	fprintf(fp, "(\"<p style = 'font-size:14px;'>country: %s<br/>city : %s</p>\");"
		                  ,city[departure_num].country,city[departure_num].city);
	fprintf(fp, "marker0.addEventListener(\"click\", function(){this.openInfoWindow(infoWindow0);}); ");
	fprintf(fp, "var marker1 = new BMap.Marker(new BMap.Point(%lf,%lf));"
	     		 ,city[way[destination_num].city[1]].longitude, city[way[destination_num].city[1]].latitude);
	fprintf(fp, "map.addOverlay(marker1);\n");
	
	for (int i = 1; i < way[destination_num].city.size(); i++)
	{
		fprintf(fp, "var infoWindow%d = new BMap.InfoWindow",i);
		fprintf(fp, "(\"<p style = 'font-size:14px;'>country: %s<br/>city : %s</p>\");"
			,city[way[destination_num].city[i]].country, city[way[destination_num].city[i]].city);
		fprintf(fp, "marker%d.addEventListener(\"click\", function(){this.openInfoWindow(infoWindow%d);}); ",i,i);
		fprintf(fp, "var contentString0%d ='",i);
		fprintf(fp, "%s,%s-->%s,%s", city[way[destination_num].city[i-1]].country, city[way[destination_num].city[i-1]].city
		                            ,city[way[destination_num].city[i]].country,city[way[destination_num].city[i]].city);
		
		//string转换为char
		fprintf(fp, "(%s-", way[destination_num].transport[i - 1].c_str());
		int route_num;  //记录是哪一条路径（由于此前优先选择的时候未在way中存储除了时间/价格的信息）
		for (route_num = 0; route_num < CityNum; route_num++)
		{
			if (way[destination_num].city[i - 1] == LocateCity(route[route_num].origin_city, key, CityNum)
				&& way[destination_num].city[i] == LocateCity(route[route_num].destin_city, key, CityNum))
				break;
		}
		fprintf(fp, "%lfhours-$%lf-", route[route_num].time, route[route_num].cost);

		fprintf(fp, "%s)';", route[route_num].info.c_str());

		fprintf(fp, "var path%d = new BMap.Polyline([new BMap.Point",i);
		fprintf(fp, "(%lf,%lf),new BMap.Point",
			city[way[destination_num].city[i-1]].longitude, city[way[destination_num].city[i-1]].latitude);
		fprintf(fp, "(%lf,%lf)],",
			city[way[destination_num].city[i]].longitude, city[way[destination_num].city[i]].latitude); 
		fprintf(fp, "{strokeColor:'#18a45b', strokeWeight:8, strokeOpacity:0.8});");
		fprintf(fp, "map.addOverlay(path%d);path%d.addEventListener", i, i);
		fprintf(fp, "(\"click\", function(){alert(contentString0%d);});", i);
		
		if (i + 1 != way[destination_num].city.size())
		{
			fprintf(fp, "var marker%d = new BMap.Marker(new BMap.Point", i);
			fprintf(fp, "(%lf,%lf));map.addOverlay(marker%d);\n"
				, city[way[destination_num].city[i]].longitude, city[way[destination_num].city[i]].latitude, i);

			fprintf(fp, "var infoWindow%d = new BMap.InfoWindow", i);
			fprintf(fp, "(\"<p style = 'font-size:14px;'>country: %s<br/>city : %s</p>\");"
				, city[way[destination_num].city[i]].country, city[way[destination_num].city[i]].city);
			fprintf(fp, "marker%d.addEventListener(\"click\", function(){this.openInfoWindow(infoWindow%d);}); ", i, i);
			fprintf(fp, "var marker%d = new BMap.Marker(new BMap.Point", i + 1);
			fprintf(fp, "(%lf,%lf));"
				, city[way[destination_num].city[i + 1]].longitude, city[way[destination_num].city[i + 1]].latitude);
			fprintf(fp, "map.addOverlay(marker%d);\n", i + 1);
		}
	}
    //中止点也需要单独写
	fprintf(fp, "</script>");

	fclose(fp);
}
int main()
{
	//计算时间//
	Timer timer;
	timer.Start();
	
	const char file_city[12] = "cities.csv";     //文件地址
	const char file_route[12] = "routes.csv";
	const char file_save[18] = "MyVisualGraph.htm";

	City my_city[CityNum];          //读取城市、路线、与城市匹配的关键字数组
	Route my_route[RouteNum];
	Key my_key[CityNum];

	char departure[30]; int departure_num;
	char destination[30]; int destination_num;
	char priority[6];             //用户选择价格/时间优先

	//printf("please enter your departure place:\n");   //getline用于读取含有空格的城市
	//cin.getline(departure,30);
	//printf("please enter your source place:\n");
	//cin.getline(destination, 30);
	//printf("please choose your priority:(TIME or PRICE):\n");
	//scanf("%s", priority);

	strcpy(departure, "Beijing");
	strcpy(destination, "Berlin");
	strcpy(priority,"TIME");           //TIME OR PRICE

	ReadCityFile(file_city, my_city, my_key);
	ReadRouteFile(file_route, my_route);
	CreateGraph(my_city, my_route, my_key, priority);

	departure_num = LocateCity(departure, my_key,CityNum);
	destination_num = LocateCity(destination, my_key, CityNum);

	Dijkistra(g, departure_num, destination_num,priority);

	Display(destination_num, my_key,priority);
	
	//DFSTraverse(my_route,my_key);    //深度优先

	VisualDisplay(my_city,my_route, departure, destination, file_save,my_key);//地图可视化
	
	timer.Stop();
	printf("\nElapsed time is: <%4.2lf> ms\n", timer.ElapsedTime());

	return 0;
}