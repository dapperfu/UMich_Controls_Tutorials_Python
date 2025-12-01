
$(window).bind('load resize',function(){

	$("#home_banner").css({
		"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top ,
		"height" : $("#ControlDigital").offset().top - $("#ControlPID").offset().top + $(".horizontal").height(),
	});
	
	$("#home_banner img").css({
		"height" : $("#home_banner").height(),
		"visibility" : "visible"
	});

	$(".vertical, .home_grid_link img, .note").css("visibility","hidden");

	if(($(".example").offset().top==($("#Introduction").offset().top+1)) && ($(".example").width() >= ($("#Introduction").width()+$("#Introduction").offset().left-$(".example").offset().left))){
		
		$("#vertical_Introduction").css({"left":$("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".vertical").width()/2 - $(".content").offset().left,"visibility":"visible"});
		//
		$("#home_grid_link_Introduction_SystemModeling").css({
			"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Introduction_SystemAnalysis").css({
			"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Introduction_ControlPID").css({
			"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Introduction_ControlRootLocus").css({
			"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Introduction_ControlFrequency").css({
			"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Introduction_ControlStateSpace").css({
			"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Introduction_ControlDigital").css({
			"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Introduction_SimulinkControl").css({
			"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
				//
		$("#home_grid_link_Introduction_SimulinkSimscape").css({
			"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Introduction_SimulinkModeling").css({
			"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		
	}
	
	if(($(".example").offset().top==($("#CruiseControl").offset().top+1)) && ($(".example").width() >= ($("#CruiseControl").width()+$("#CruiseControl").offset().left-$(".example").offset().left))){
		
		$("#vertical_CruiseControl").css({"left":$("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".vertical").width()/2 - $(".content").offset().left,"visibility":"visible"});
		//
		$("#home_grid_link_CruiseControl_SystemModeling").css({
			"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_SystemAnalysis").css({
			"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_ControlPID").css({
			"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_ControlRootLocus").css({
			"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_ControlFrequency").css({
			"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_ControlStateSpace").css({
			"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_ControlDigital").css({
			"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_SimulinkControl").css({
			"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_SimulinkSimscape").css({
			"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_CruiseControl_SimulinkModeling").css({
			"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#CruiseControl").offset().left + $("#CruiseControl").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		
	}
	
	if(($(".example").offset().top==($("#MotorSpeed").offset().top+1)) && ($(".example").width() >= ($("#MotorSpeed").width()+$("#MotorSpeed").offset().left-$(".example").offset().left))){
		
		$("#vertical_MotorSpeed").css({"left":$("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".vertical").width()/2 - $(".content").offset().left,"visibility":"visible"});
		//
		$("#home_grid_link_MotorSpeed_SystemModeling").css({
			"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_SystemAnalysis").css({
			"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_ControlPID").css({
			"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_ControlRootLocus").css({
			"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_ControlFrequency").css({
			"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_ControlStateSpace").css({
			"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_ControlDigital").css({
			"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_SimulinkControl").css({
			"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_SimulinkSimscape").css({
			"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorSpeed_SimulinkModeling").css({
			"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorSpeed").offset().left + $("#MotorSpeed").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
	
	}
	
	if(($(".example").offset().top==($("#MotorPosition").offset().top+1)) && ($(".example").width() >= ($("#MotorPosition").width()+$("#MotorPosition").offset().left-$(".example").offset().left))){
		
		$("#vertical_MotorPosition").css({"left":$("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".vertical").width()/2 - $(".content").offset().left,"visibility":"visible"});
		//
		$("#home_grid_link_MotorPosition_SystemModeling").css({
			"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_SystemAnalysis").css({
			"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_ControlPID").css({
			"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_ControlRootLocus").css({
			"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_ControlFrequency").css({
			"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_ControlStateSpace").css({
			"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_ControlDigital").css({
			"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_SimulinkControl").css({
			"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_SimulinkSimscape").css({
			"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_MotorPosition_SimulinkModeling").css({
			"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#MotorPosition").offset().left + $("#MotorPosition").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		
	}
	
	if(($(".example").offset().top==($("#Suspension").offset().top+1)) && ($(".example").width() >= ($("#Suspension").width()+$("#Suspension").offset().left-$(".example").offset().left))){
		
		$("#vertical_Suspension").css({"left":$("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".vertical").width()/2 - $(".content").offset().left,"visibility":"visible"});
		//
		$("#home_grid_link_Suspension_SystemModeling").css({
			"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_SystemAnalysis").css({
			"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_ControlPID").css({
			"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_ControlRootLocus").css({
			"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_ControlFrequency").css({
			"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_ControlStateSpace").css({
			"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_ControlDigital").css({
			"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_SimulinkControl").css({
			"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_SimulinkSimscape").css({
			"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_Suspension_SimulinkModeling").css({
			"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#Suspension").offset().left + $("#Suspension").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		
	}
	
	if(($(".example").offset().top==($("#InvertedPendulum").offset().top+1)) && ($(".example").width() >= ($("#InvertedPendulum").width()+$("#InvertedPendulum").offset().left-$(".example").offset().left))){
		
		$("#vertical_InvertedPendulum").css({"left":$("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".vertical").width()/2 - $(".content").offset().left,"visibility":"visible"});
		//
		$("#home_grid_link_InvertedPendulum_SystemModeling").css({
			"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_SystemAnalysis").css({
			"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_ControlPID").css({
			"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_ControlRootLocus").css({
			"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_ControlFrequency").css({
			"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_ControlStateSpace").css({
			"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_ControlDigital").css({
			"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_SimulinkControl").css({
			"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_SimulinkSimscape").css({
			"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_InvertedPendulum_SimulinkModeling").css({
			"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#InvertedPendulum").offset().left + $("#InvertedPendulum").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		
	}
	
	if(($(".example").offset().top==($("#AircraftPitch").offset().top+1)) && ($(".example").width() >= ($("#AircraftPitch").width()+$("#AircraftPitch").offset().left-$(".example").offset().left))){
		
		$("#vertical_AircraftPitch").css({"left":$("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".vertical").width()/2 - $(".content").offset().left,"visibility":"visible"});
		//
		$("#home_grid_link_AircraftPitch_SystemModeling").css({
			"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_SystemAnalysis").css({
			"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_ControlPID").css({
			"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_ControlRootLocus").css({
			"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_ControlFrequency").css({
			"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_ControlStateSpace").css({
			"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_ControlDigital").css({
			"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_SimulinkControl").css({
			"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_SimulinkSimscape").css({
			"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_AircraftPitch_SimulinkModeling").css({
			"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#AircraftPitch").offset().left + $("#AircraftPitch").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		
	}
	
	if(($(".example").offset().top==($("#BallBeam").offset().top+1)) && ($(".example").width() >= ($("#BallBeam").width()+$("#BallBeam").offset().left-$(".example").offset().left))){
		
		$("#vertical_BallBeam").css({"left":$("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".vertical").width()/2 - $(".content").offset().left,"visibility":"visible"});
		//
		$("#home_grid_link_BallBeam_SystemModeling").css({
			"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_SystemAnalysis").css({
			"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_ControlPID").css({
			"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_ControlRootLocus").css({
			"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_ControlFrequency").css({
			"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_ControlStateSpace").css({
			"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_ControlDigital").css({
			"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_SimulinkControl").css({
			"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_SimulinkSimscape").css({
			"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		//
		$("#home_grid_link_BallBeam_SimulinkModeling").css({
			"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".content").offset().top - 7,
			"left" : $("#BallBeam").offset().left + $("#BallBeam").outerWidth()/2 - $(".content").offset().left - 7, 
			"visibility" : "visible"
		});
		
	}
	
	//

	$("#note_top_tabs").css({
		"top" : $("#System").offset().top + 5 - $(".content").offset().top,
		"left" : $("#CruiseControl").offset().left - $("#note_top_tabs").outerWidth()/2 - $(".content").offset().left + 1,
		"visibility" : "visible"
	});	
	$("#note_top_tabs img").css({
		"left" : $("#note_top_tabs").outerWidth()/2 - $("#note_top_tabs img").outerWidth()/2
	});

	$("#note_left_tabs").css({
		"top" : $("#SimulinkControl").offset().top - $("#note_left_tabs").outerHeight()/2 - $(".content").offset().top,
		"left" : $("#SimulinkControl").offset().left + $("#SimulinkModeling").outerWidth()  - $(".content").offset().left + 5,
		"visibility" : "visible"
	});
	$("#note_left_tabs img").css({
		"top" : $("#note_left_tabs").outerHeight()/2 - $("#note_left_tabs img").outerHeight()/2
	});

	$("#note_grid_links").css({
		"top" : $("#ControlDigital").offset().top + 35 - $(".content").offset().top,
		"left" : $("#Introduction").offset().left + $("#Introduction").outerWidth()/2 - $("#note_grid_links").outerWidth()/2  - $(".content").offset().left + 1,
		"visibility" : "visible"
	});
	$("#note_grid_links img").css({
		"left" : $("#note_grid_links").outerWidth()/2 - $("#note_grid_links img").outerWidth()/2
	});
	
	$("#note_home").css({
		"top" : $("#System").offset().top + 5 - $(".content").offset().top,
		"left" : $("#CTMS_logo").offset().left + $("#CTMS_logo").outerWidth()*3/4 - $("#note_home").outerWidth()/2  - $(".content").offset().left + 1,
		"visibility" : "visible"
	});
	$("#note_home img").css({
		"left" : $("#note_home").outerWidth()/2 - $("#note_home img").outerWidth()/2
	});

	$("#note_index").css({
		"top" : $("#Tips").offset().top + $("#Tips").outerHeight() + 5 - $(".content").offset().top,
		"left" : $("#Tips").offset().left + $("#Tips").outerWidth()/2 - $("#note_index").outerWidth()/2  - $(".content").offset().left + 1,
		"visibility" : "visible"
	});
	$("#note_index img").css({
		"left" : $("#note_index").outerWidth()/2 - $("#note_index img").outerWidth()/2
	});

	$("#note_next").css({
		"top" : $("#Next").offset().top + $("#Next").outerHeight() + 5 - $(".content").offset().top,
		"left" : $("#Next").offset().left + $("#Next").outerWidth()/2 - $("#note_next").outerWidth()/2  - $(".content").offset().left + 1,
		"visibility" : "visible"
	});
	$("#note_next img").css({
		"left" : $("#note_next").outerWidth()/2 - $("#note_next img").outerWidth()/2
	});
	
	if($("#container").width() < 1177){
		$("#note_more_examples").css({
			"top" : $("#Introduction").offset().top + $("#Introduction").outerHeight() + 5 - $(".content").offset().top,
			"left" : $("#container").offset().left + $("#container").outerWidth() - $("#note_more_examples").outerWidth()/2 - $(".content").offset().left - 13.5,
			"visibility" : "visible"
		});
		$("#note_more_examples img").css({
			"left" : $("#note_more_examples").outerWidth()/2 - $("#note_more_examples img").outerWidth()/2
		});
	}
		
});	

$(window).bind('load',function(){

	$(".section_heading_arrow").attr("src","Content/Home/figures/home_section_heading_arrow.png");
	$(".section_heading_arrow").css("opacity",1);
	$(".section_arrow").attr("src","Content/Home/figures/home_section_arrow.png");
	$(".section_arrow").css("opacity",0.3);
	
	$("#horizontal_System").css({"top":$("#System").offset().top + $(".section .heading").outerHeight()/2 - $(".horizontal").height()/2  - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_Control").css({"top":$("#Control").offset().top + $(".section .heading").outerHeight()/2 - $(".horizontal").height()/2  - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_Simulink").css({"top":$("#Simulink").offset().top + $(".section .heading").outerHeight()/2 - $(".horizontal").height()/2  - $(".content").offset().top,"visibility":"visible"});
	
	$("#horizontal_SystemModeling").css({"top" : $("#SystemModeling").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_SystemAnalysis").css({"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_ControlPID").css({"top" : $("#ControlPID").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_ControlRootLocus").css({"top" : $("#ControlRootLocus").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_ControlFrequency").css({"top" : $("#ControlFrequency").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_ControlStateSpace").css({"top" : $("#ControlStateSpace").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_ControlDigital").css({"top" : $("#ControlDigital").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_SimulinkModeling").css({"top" : $("#SimulinkModeling").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_SimulinkSimscape").css({"top" : $("#SimulinkSimscape").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	$("#horizontal_SimulinkControl").css({"top" : $("#SimulinkControl").offset().top + $(".section .item").outerHeight()/2 - $(".horizontal").height()/2 - $(".content").offset().top,"visibility":"visible"});
	
	$("#vertical_offset").css({
		"top" : $("#SystemAnalysis").offset().top + $(".section .item").outerHeight()/2 + $(".horizontal").height()/2 - $(".content").offset().top
	});

	$("#home_CTMS_logo").css({
		"padding-left" : $("#vertical_CruiseControl").offset().left + $(".vertical").width() - $("#home_CTMS_logo").width() - $(".content").offset().left - 5,
		"padding-top" : ($("#ControlPID").offset().top - $("#SystemAnalysis").offset().top - $(".horizontal").height() - $("#home_CTMS_logo").height())/2,
		"padding-bottom" : ($("#ControlPID").offset().top - $("#SystemAnalysis").offset().top - $(".horizontal").height() - $("#home_CTMS_logo").height())/2,
		"visibility":"visible"
	});

	$("#home_text_welcome").css({
		"height" : $("#home_CTMS_logo").outerHeight() - 12,
		"padding-left": $("#home_CTMS_logo").outerWidth() + 12,
		"visibility":"visible"
	});

	$("#home_text_authors").css({
		"height" : $("#home_CTMS_logo").outerHeight() - 12,
		"top" : $("#ControlDigital").offset().top - $("#ControlPID").offset().top + $(".horizontal").height(),
		"margin-left": $("#home_CTMS_logo").outerWidth(),
		"visibility":"visible"	
	});
		
});

// Define animation sequence on home page load

var speed_1 = 1000; // Speed of 1st animation step (grid and grid links) [ms]
var speed_2 = 750; // Speed of 2nd animation step (CTMS logo and text boxes) [ms]
var speed_3 = 750; // Speed of 3rd animation step (home naviagation notes) [ms]

$(".home_grid_link img").animate({
	opacity: 1
	}, speed_1, function(){
});

$(".horizontal_heading").animate({
	opacity: 1
	}, speed_1, function(){
});

$("#horizontal_SystemModeling, #horizontal_SystemAnalysis, #horizontal_SimulinkModeling, #horizontal_SimulinkSimscape, #horizontal_SimulinkControl").animate({
	opacity: 0.7
	}, speed_1, function(){
});

$("#horizontal_ControlPID, #horizontal_ControlRootLocus, #horizontal_ControlFrequency, #horizontal_ControlStateSpace, #horizontal_ControlDigital").animate({
	opacity: 0.3
	}, speed_1, function(){
});

$(".vertical").animate({
	opacity: 0.5
	}, speed_1, function(){
	
	$("#home_CTMS_logo, #home_text_welcome, #home_text_authors").animate({
	opacity: 1
	}, speed_2, function(){
	});

	$("#home_banner img").animate({
	opacity: 0.8
	}, speed_2, function(){
	
		$(".note").animate({
		opacity: 1
		}, speed_3, function(){
		});
	
	});
	
});

// Grid link roll over effect

$(".home_grid_link img").hover(
  function () {
	$(this).css("box-shadow","0px 0px 3px 2px rgba(30,96,165,1)");
  }, 
  function () {
	$(this).css("box-shadow","none");
  }
);
