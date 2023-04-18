
void pageModeStart(unsigned int num)
{
	unsigned int flag=0;
	setNumPages(num);
//	setSizeSamples(num*1024);//Peter fix
//	pageMode(1);
	flag=readEnWrite()>>31;
	while(flag==0)
	{
		printf("flag=%d\n",flag);
		readExternalStatus(0x3d);
		flag=readEnWrite()>>31;
	}
    adc_set_flag_end_read(-1, true);
	readExternalStatus(0x3d);
}

/**
 * unsigned short *buffer		буфер данных;
 * unsigned int *shift			указатель на  unsigned int в который положится сдвиг;
 * unsigned short extStart		индикатор внешнего запуска;
 */
unsigned int onceGet(unsigned short *buffer,unsigned int *shift,unsigned int calibrate,unsigned int extStart,unsigned int drsnum)
{
	unsigned int flag=0,i=0,end=0;
	if(extStart==0)
	{
		setWorkDRS(1);
		usleep(100);
		if(calibrate==0)
		{
			softStartRecorder(1);
		}
	}
	flag=readEnWrite()>>31;
	while(flag==0 && end==0)
	{
		flag=readEnWrite()>>31;
		i++;
		if(extStart==0)
		{
			if(i>100)
			{
				end=1;
			}
		}else{
			//if(ext_start==0){end=1;)
		}
		//readExternalStatus(0xc); //Peter fix
	}
	if(flag==1){
        readNPage(&buffer[0],&shift[0],0,drsnum);
//		readNPage(&buffer[8192],&shift[1024],0,1,drsnum);
	}
	flagEndRead(1);
	return flag;
}

/**
 * unsigned int drsnum		номер drs для вычитывания сдвига
 * return 					индекс сдвига;
 */
unsigned int getShiftIndex(unsigned int drsnum)//npage
{
    unsigned short tmpshift;
    if (drsnum==0)
     tmpshift=((unsigned long *)data_map_shift_drs1)[0]&1023;
    else
     tmpshift=((unsigned long *)data_map_shift_drs2)[0]&1023;
	return tmpshift;
}

/**
 * double *average					масиив со средними значениями каналов;
 * double *data						массив данных
 * unsigned int chanalLength		длинна канала;
 * unsigned int chanalCount			число каналов;
 */
void getAverage(double *average,double *data,unsigned int chanalLength,unsigned int chanalCount)
{
    unsigned int i,j;
	for(i=0;i<chanalCount;i++)
	{
		average[i]=0.0;
		for(j=0;j<chanalLength;j++)
		{
			average[i]+=data[j*chanalCount+i];
		}
		average[i]/= (double) chanalLength;
		//printf("average[%d]=%f\n",i,average[i]);
	}
}

/**
 * @brief adc_ch_get_average
 * @param a_cells
 * @param a_cells_count
 * @param a_ch_id
 * @return
 */
double adc_ch_get_average(double *a_cells, unsigned a_cells_count, unsigned a_ch_id)
{
    double l_acc=0.0;
    double l_min=0.0, l_max=0.0;
    for(unsigned n=0; n<a_cells_count ; n++){
        unsigned l_idx = n* adc_CHANNELS_COUNT + a_ch_id;
        double l_cell = a_cells[l_idx];
        l_acc += l_cell;
        if(l_cell < l_min)
            l_min = l_cell;
        if(l_cell > l_max)
            l_max = l_cell;
    }

    double l_ret = l_acc / ((double) a_cells_count);

    debug_if(s_debug_more,L_DEBUG,"adc_ch_get_average: l_acc=%f, l_ret=%f, l_min=%f, l_max=%f, a_cells_count=%u",
             l_acc, l_ret, l_min, l_max, a_cells_count);

    return l_ret;
}
/**
 * double *         a_average		масиив со средними значениями каналов;
 * unsigned short*  a_cells		массив данных
 * unsigned int     a_ch_cells_count	длинна канала;
 * unsigned int     a_ch_count		число каналов;
 */
void getAverageInt(double *a_average,unsigned short *a_cells,unsigned int a_ch_cells_count,unsigned int a_ch_count)
{
    unsigned int i,j;
    for(i=0;i<a_ch_count;i++){
        a_average[i]=0;
        for(j=0;j<a_ch_cells_count;j++){
                a_average[i]+=a_cells[j*a_ch_count+i];
        }
        a_average[i]/= (double) a_ch_cells_count;
        //printf("average[%d]=%f\n",i,average[i]);
    }
}

 double absf(double value)
 {
	 if(value<0)
	 {
		 return value*-1;
	 }else{
		 return value;
	 }
 }

 /**
  * unsigned short *buffer		массив данных
  * unsigned int *shift			сдвиг;
  * unsigned int pageCount		номер страницы;
  */
 void readNPage(unsigned short *buffer,unsigned int* shift,unsigned int numPage, unsigned int drsnum)
 {
    if (drsnum==0)
        memcpy(buffer, &(((unsigned short *)data_map_drs1)[numPage*adc_CELLS_COUNT_ALL*adc_CHANNELS_COUNT]), adc_PAGE_ALL_SIZE  );
  	else
        memcpy(buffer, &(((unsigned short *)data_map_drs2)[numPage*adc_CELLS_COUNT_ALL*adc_CHANNELS_COUNT]), adc_PAGE_ALL_SIZE);
        *shift=getShiftIndex(drsnum);
 }

 /**
  * unsigned short *buffer		массив данных
  * unsigned int *shift			сдвиг;
  * unsigned int pageCount		номер страницы;
  */
 void writeNPage(unsigned short *buffer,unsigned int numPage, unsigned int drsnum)
 {
    if (drsnum==0)
 	 memcpy(&(((unsigned short *)data_map_drs1)[numPage*16384]),buffer, 0x8000);
 	else
 	 memcpy(&(((unsigned short *)data_map_drs2)[numPage*16384]),buffer, 0x8000);
 }
/**
 * unsigned short *buffer		массив данных
 * unsigned int *shift			сдвиги;
 * unsigned int pageCount		число страниц;
 */
void readNPages(unsigned short *buffer,unsigned int *shift,unsigned int pageCount, unsigned int step, unsigned int drsnum)
{
     unsigned int t;
	 for (t=0; t<pageCount;t++)
	 {
         readNPage(&buffer[t*step],&shift[t],t, drsnum);
	 }
}



