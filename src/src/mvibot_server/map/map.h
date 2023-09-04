#include "../mvibot_server_init.h"
using namespace  std;
void pub_map(){
    static ros::NodeHandle n;
    static ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>( "/map2", 1 );
    static float creat_fun=0;
	if(creat_fun==1)
	{
        static int have_to_pub;
        static uint32_t n=0;
        static nav_msgs::OccupancyGrid map_var;
        have_to_pub=0;
        if(request_map_topic==1){
            request_map_topic=0;
            have_to_pub=1;
        }
        if(map_pub.getNumSubscribers()!=n){
            n=map_pub.getNumSubscribers();
            have_to_pub=1;
        }
        for(int i=0;i<my_layers.size();i++){
            if(my_layers[i].is_have==0) {
                have_to_pub=1;
                my_layers.erase(my_layers.begin()+i);
                i--;
            }
        }
        for(int i=0;i<my_layers.size();i++){
            //
            if(my_layers[i].is_pub==0) {
                my_layers[i].is_pub=1;
                have_to_pub=1;
            }
        }
        if(have_to_pub==1){
            have_to_pub=0;
            for(int i=0;i<my_map.data.size();i++) my_map.data[i]=-1;
            map_var=my_map;
            for(int i=0;i<map_var.data.size();i++)
            {
                static float x,y;
                static float x_origin,y_origin,yaw_origin;
                x_origin=map_var.info.origin.position.x;
                y_origin=map_var.info.origin.position.y;
                yaw_origin=tf::getYaw(map_var.info.origin.orientation);
                // convert to x y origin
                x=(float)(i-(uint32_t)(i/map_var.info.width)*map_var.info.width)*map_var.info.resolution;
                y=(float)((uint32_t)(i/map_var.info.width))*map_var.info.resolution;
                //
                x=x+x_origin;
                y=y+y_origin;
                //
                static float dis;
                dis=sqrt(x*x+y*y);
                static float ampha;
                ampha=atan2(y,x);
                //
                ampha=ampha+yaw_origin;
                x=dis*cos(ampha);
                y=dis*sin(ampha);
                // check cost map layer
                for(int j=0;j<my_layers.size();j++){
                    if(name_map_active==my_layers[j].name_map_active){
                        static float x_layer,y_layer,yaw_layer;
                        x_layer=x-my_layers[j].xo;
                        y_layer=y-my_layers[j].yo;
                        yaw_layer=0-my_layers[j].yawo;
                        //
                        static float dis_layer;
                        dis_layer=sqrt(x_layer*x_layer+y_layer*y_layer);
                        //
                        static float ampha_layer;
                        ampha_layer=atan2(y_layer,x_layer);
                        ampha_layer=ampha_layer+yaw_layer;
                        x_layer=dis_layer*cos(ampha_layer);
                        y_layer=dis_layer*sin(ampha_layer);
                        //
                        if(fabs(x_layer)<=my_layers[j].width/2 &  fabs(y_layer)<=my_layers[j].heigth/2) {
                            static float vlaue_cost;
                            if(my_layers[j].type_layer=="dead_zone") vlaue_cost=100;
                            if(my_layers[j].type_layer=="high_zone") vlaue_cost=95;
                            if(map_var.data[i] < vlaue_cost) map_var.data[i]=vlaue_cost;
                        }
                    }
                }
                //
            }
            map_pub.publish(map_var);   
        }
	} else creat_fun=1;
}
nav_msgs::OccupancyGrid get_map_select(string path){
    static nav_msgs::OccupancyGrid map_msg;
    try{
        //
        map_msg.header.frame_id="map";
        static geometry_msgs::Pose pose_origin;
        static float occupied_thresh,free_thresh,negate,resolution;
        // get info from yaml
        std::ifstream file(path+".yaml");
	    static std::string str; 
        static string_Iv2 data;
	    while (std::getline(file, str))
	    {
            cout<<str<<endl;
            data.detect(str,"",": ","");
            if(data.data1[0]=="resolution") resolution=stof(data.data1[1]);
            if(data.data1[0]=="negate") negate=stof(data.data1[1]);
            if(data.data1[0]=="occupied_thresh") occupied_thresh=stof(data.data1[1]);
            if(data.data1[0]=="free_thresh") free_thresh=stof(data.data1[1]);
            if(data.data1[0]=="origin") {
               // pose_origin
               static string_Iv2 data2;
               data2.detect(data.data1[1],"[","","]");
               static string_Iv2 data3;
               data3.detect(data2.data1[0],"",", ","");
               //
               pose_origin.position.x=stof(data3.data1[0]);
               pose_origin.position.y=stof(data3.data1[1]);
               pose_origin.orientation=tf::createQuaternionMsgFromYaw(stof(data3.data1[2]));
            }
	    }
        file.close();
        map_msg.info.resolution=resolution;
        map_msg.info.origin=pose_origin;
        // get info from pgm
        int row = 0, col = 0, num_of_rows = 0, num_of_cols = 0;
        stringstream ss; 
        ifstream infile(path+".pgm", ios::binary);
        string inputLine = "";
        getline(infile,inputLine); // read the first line : P5
        if(inputLine.compare("P5") != 0) cerr << "Version error" << endl;
        cout << "Version : " << inputLine << endl;
        getline(infile,inputLine); // read the second line : comment
        cout << "Comment : " << inputLine << endl;
        ss << infile.rdbuf(); //read the third line : width and height
        ss >> num_of_cols >> num_of_rows;
        cout << num_of_cols << " columns and " << num_of_rows << " rows" << endl;
        int max_val; //maximum intensity value : 255
        ss >> max_val;
        cout<<max_val<<endl;
            map_msg.info.height=num_of_rows;
            map_msg.info.width=num_of_cols;
            map_msg.data.resize(num_of_rows*num_of_cols);
        //
        static int n;
        for(int i=num_of_rows-1;i>=0;i--){
            for(int j=num_of_cols-1;j>=0;j--){
                static unsigned char pixel;
                ss>>pixel;
                //n--;
                n=i*num_of_cols+(num_of_cols-1-j);
                //cout<<n<<endl;
                if(negate==0){
                    map_msg.data[n]=-1;
                    //
                    static float soc;
                    soc=(float)(255-(float)pixel)/255;
                    if(soc>occupied_thresh) map_msg.data[n]=100;
                    if(soc<free_thresh) map_msg.data[n]=0;
                }else{
                    map_msg.data[n]=-1;
                    //
                    static float soc;
                    soc=(float)(255-(float)pixel)/255;
                    if(soc>occupied_thresh) map_msg.data[n]=100;
                    if(soc<free_thresh+0.1) map_msg.data[n]=0;
                }
            }
        }
        //
        infile.close();
    }catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return map_msg;
}