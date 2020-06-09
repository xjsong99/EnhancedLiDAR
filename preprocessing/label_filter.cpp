#include <bits/stdc++.h>
#include <utility>
#include <dirent.h>
using namespace std;

FILE *fin,*fout;

char data[205];

int fileNameFilter_txt(const struct dirent *cur)
{
    string str(cur->d_name);
    if(str.find(".txt")!=std::string::npos)
        return 1;
    return 0;
}

int main(int argc, char **argv)
{
    struct dirent **namelist_gt, **namelist_pr; //文件名list

    string label_path(argv[1]);//= "/home/song/下载/pointrcnn训练结果/epoch_no_number/val/final_result/data/";
    string save_path(argv[2]);//= "/home/song/下载/pointrcnn训练结果/epoch_no_number/val/final_result/data-30m/";

    int num_of_file = scandir(label_path.c_str(), &namelist_gt, fileNameFilter_txt, alphasort);

    printf("num_of_file=%d\n",num_of_file);

    char class_name[35];
    float fdata[17],dis_threshold;
    sscanf(argv[3],"%f",&dis_threshold);

    for (int file_index = 0; file_index < num_of_file; file_index++)
    {
        printf("[%d]:%s\n",file_index,namelist_gt[file_index]->d_name);
        fin = fopen ((label_path+string(namelist_gt[file_index]->d_name)).c_str(),"r");
        fout = fopen ((save_path+string(namelist_gt[file_index]->d_name)).c_str(),"w");

        while(fgets(data,200,fin))
        {
            sscanf(data,"%s %f %f %f %f %f %f %f %f %f %f %f %f %f",
                   class_name,&fdata[2],&fdata[3],&fdata[4],&fdata[5],&fdata[6],&fdata[7],&fdata[8],&fdata[9],&fdata[10],&fdata[11],&fdata[12],&fdata[13],&fdata[14]);
            if(fdata[14]>dis_threshold) continue;
            fprintf(fout,"%s",data);
        }

        fclose(fin);
        fclose(fout);
    }

    return 0;
}
