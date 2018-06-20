


// ==================================================================================================
// Local Parameters
// ==================================================================================================

// Icons
var robot_icons = {
	'glider':	new ol.style.Icon({src: 'ico/glider.svg'}),
	'kayak':	new ol.style.Icon({src: 'ico/kayak.svg'}),
}

// Labels
var robot_label = new ol.style.Text({
	textAlign: 'left',
	textBaseline: 'top',
	fill: new ol.style.Fill({color: 'black'}),
	offsetX: 15,
	offsetY: 15,
	rotation: 0,
})

// ==================================================================================================
// ROS Parameter Requests
// ==================================================================================================

// Params
$.ajax({
	async: false,
	method: 'POST',
	url: '',
	data: 'params',
	dataType: 'json',
	success: function(data, status){params=data;},
});
var wms_server_url = params.wms_server.protocol+'://'+params.wms_server.ip+':'+params.wms_server.port+params.wms_server.datapath;


(function fetch_robots(){
	$.ajax({
		method: 'POST',
		url: '',
		data: 'robots',
		dataType: 'json',
		success: get_robots,
		complete: function(){
			setTimeout(fetch_robots, params.web_viz.update_rate);
		},
	});
})();

// ==================================================================================================
// Local WMS Server Parameter Requests
// ==================================================================================================


// Offline Tile Server Capabilities
var get_wms_server_capabilities = function(url){
	var ret;
	$.ajax({
		async: false,
		method: 'GET',
		url: wms_server_url,
		data: {SERVICE: 'WMTS', REQUEST: 'GetCapabilities'},
		dataType: 'xml',
		success: function(data, status){ret=data},
	});
	var parser = new ol.format.WMSCapabilities();
	return parser.read(ret).Capability.Layer.Layer;
}

// Layer Styles
var get_tile_styles = function(layer_code){
	var ret;
	$.ajax({
		async: false,
		method: 'GET',
		url: wms_server_url,
		data: {request: 'GetMetadata', item: 'layerDetails', layerName: layer_code},
		dataType: 'json',
		success: function(data, status){ret=data},
	});
	return ret;
}



// ==================================================================================================
// Layer Setup
// ==================================================================================================

var layer_collection	= new ol.Collection();
var layer_styles = {};

// Refreshing Layers
(function(){
	layer_collection.forEach(function(element, index, array){
		if(element instanceof ol.layer.Tile){
			if(typeof element.getSource === 'function' && typeof element.getSource().updateParams === 'function'){
				element.getSource().updateParams({'validtime': (new Date()).getTime()});
			}
		}
	});
	setTimeout(arguments.callee, params.web_viz.update_raster_ms);
})();

var get_vector_layers = function(){
	for(var p in params.vector_layers){
		var layer_name = params.vector_layers[p].title
		layer_collection.push(new ol.layer.Vector({
			title: layer_name,
			visible: params.vector_layers[p].visible,
			source: new ol.source.Vector({
				url: params.vector_layers[p].url,
				format: new ol.format.GeoJSON(),
			}),
			style: new ol.style.Style({
				fill: new ol.style.Fill({color: params.vector_layers[p].fill}),
				stroke: new ol.style.Stroke ({
					color: params.vector_layers[p].stroke.color,
					lineCap: params.vector_layers[p].stroke.lineCap,
					lineJoin: params.vector_layers[p].stroke.lineJoin,
					lineDash: params.vector_layers[p].stroke.lineDash,
					lineDashOffset: params.vector_layers[p].stroke.lineDashOffset,
					miterLimit: params.vector_layers[p].stroke.miterLimit,
					width: params.vector_layers[p].stroke.width,
				}),
			}),
		}));
	}
}

var get_offline_tile_layers = function(){
	var datasets = get_wms_server_capabilities();
	for(d in datasets){
		for(l in datasets[d].Layer){
			var layer		= datasets[d].Layer[l];
			var layer_name	= datasets[d].Title+': '+layer.Title;
			var layer_code	= layer.Name;
			var format		= layer.Style[0].Format;
			var properties	= {
				title: layer_name,
				opacity: 1.0,
				visible: false,
				preload: params.web_viz.preload,
				source: new ol.source.TileWMS({
					url: wms_server_url,
					validtime: (new Date()).getTime(),
					params: {
						REQUEST: 'GetMap',
						CRS: params.web_viz.bkg_projection,
						FORMAT: format,
						STYLES: 'default-scalar/default',
						LAYERS: layer_code,
						ABOVEMAXCOLOR: params.web_viz.beyond_limit_color,
						BELOWMINCOLOR: params.web_viz.beyond_limit_color, 
					}
				})
			};
			layer_collection.push(new ol.layer.Tile(properties));
			var index = layer_collection.getLength()-1;
			layer_styles[index] = layer;
			layer_styles[index].tile_styles = get_tile_styles(layer_styles[index].Name);
		}
	}
}

var get_online_tile_layers = function(){
	for(var p in params.online_layers){
		var layer_name = params.online_layers[p].title;
		var properties = {
			title: layer_name,
			visible: params.online_layers[p].visible,
			source: new ol.source.XYZ({
				url: params.online_layers[p].url,
			})
		}
		layer_collection.push(new ol.layer.Tile(properties));
	}
}

var robot_features = new ol.Collection();
layer_collection.push(new ol.layer.Vector({
	title: 'robotLayer',
	source: new ol.source.Vector({
		features: robot_features
	}),
}));

get_vector_layers();
get_offline_tile_layers();
get_online_tile_layers();



// ==================================================================================================
// Map Setup
// ==================================================================================================

var map = new ol.Map({
	layers: layer_collection,
	target: 'map',
	view: new ol.View({
		maxZoom: params.web_viz.max_zoom,
		center: ol.proj.fromLonLat(params.web_viz.origin_lat_lon,params.web_viz.bkg_projection),
		projection: params.web_viz.bkg_projection,
		zoom: params.web_viz.initial_zoom,
	})
});

// ==================================================================================================
// OpenLayers Standard Controls
// ==================================================================================================

var coordinate_format = function(coordinate){
	if(params.web_viz.coordinate_format == 'XY'){
		return ol.coordinate.toStringXY(ol.proj.transform(coordinate, params.web_viz.bkg_projection, 'EPSG:4326'), params.web_viz.coordinate_digits);
	}
	if(params.web_viz.coordinate_format == 'HDMS'){
		return ol.coordinate.toStringHDMS(ol.proj.transform(coordinate, params.web_viz.bkg_projection, 'EPSG:4326'), params.web_viz.coordinate_digits);
	}
}
map.addControl(new ol.control.FullScreen());
map.addControl(new ol.control.ZoomSlider());
map.addControl(new ol.control.ZoomToExtent());
map.addControl(new ol.control.ScaleLine({
	units: params.web_viz.scale_units,
	minWidth: params.web_viz.scale_width,
}
));
map.addControl(new ol.control.MousePosition({
	coordinateFormat: coordinate_format,
}
));
map.addControl(new ol.control.OverviewMap());


// ==================================================================================================
// Popups
// ==================================================================================================

var popup = new ol.Overlay.Popup();
map.addOverlay(popup);
map.on('singleclick', function(evt) {
	popup.show(evt.coordinate, '<div><p>' + coordinate_format(evt.coordinate) + '</p></div>');
});

// ==================================================================================================
// Updating robot position function
// ==================================================================================================

var get_robots = function(data, status){
	robot_features.clear();
	for(var r in data){
		var label = robot_label.clone();
		label.setText(data[r].name);
		var nf = new ol.Feature({geometry: new ol.geom.Point(ol.proj.fromLonLat(data[r].pos,params.web_viz.bkg_projection))});
		nf.setStyle(new ol.style.Style({
			image: robot_icons[data[r].type],
			text: label,
		}))
		robot_features.push(nf);
	}
};


// ==================================================================================================
// Layer Style and Selection Drop-Down Menus
// ==================================================================================================


var update_legend = function(){
	var index = $('#layer-settings').attr('name');
	var	sty = layer_collection.item(index).getSource().getParams().STYLES;
	var	layers = layer_collection.item(index).getSource().getParams().LAYERS;
	$('#legend>img').attr('src',wms_server_url+'?REQUEST=GetLegendGraphic&LAYERS='+layers+'&STYLES='+sty+'&WIDTH=10&HEIGHT=300');
}

var update_sty = function(){
	var index = $('#layer-settings').attr('name');
	var sty = $('#style-sel').find(":selected").val()+'/'+$('#palettes-sel').find(":selected").val();
	layer_collection.item(index).getSource().updateParams({'STYLES':sty});
	layer_collection.item(index).setOpacity($('#opacity').val()*0.01);
	update_legend();
}

// Menu Button
$('div.map>div.ol-viewport>div.ol-overlaycontainer-stopevent').append($('<div>', {id:'ol-main-menu-pad',class:'ol-main-menu'}));
$('#ol-main-menu-pad').append($('<div>', {id:'ol-main-menu-button-open',class:'ol-main-menu-button ol-main-menu-button-open'}));
$('#ol-main-menu-button-open').click(
	function(){
		$('#ol-main-menu-pad').hide();
		$('#ol-main-menu').show();
	}
);

// Main Menu Panel
$('div.map>div.ol-viewport>div.ol-overlaycontainer-stopevent').append($('<div>', {id:'ol-main-menu',class:'ol-main-menu'}));
$('#ol-main-menu').hide();
$('#ol-main-menu').append($('<div>', {id:'ol-main-menu-button-close',class:'ol-main-menu-button ol-main-menu-button-close'}));
$('#ol-main-menu-button-close').click(
	function(){
		$('#ol-main-menu-pad').show();
		$('#ol-main-menu').hide();
		$('#layer-settings').hide();
		$('#dataset-sel').val([]);
		$('#variable-sel').empty()
		$('#palettes-sel').empty()
		$('#style-sel').empty()
		$('#range-sels').remove();
	}
);

// Layer Sorting
$('#ol-main-menu').append($('<div>', {id:'sorting'}));
$('#sorting').append($('<ul>', {id:'sortable'}));
layer_collection.forEach(function(element, index, array){
	$('#sortable').append($('<li>', {id:index, class:'ui-state-default'}));
	$('#'+index).append($('<input>', {id:index+'-checkbox',type:'checkbox',checked:layer_collection.item(index).getVisible()}));
	$('#'+index+'-checkbox').change(function(){
		layer_collection.item(index).setVisible($(this).prop('checked'))
	});
	element.setZIndex(-index);
	$('#'+index).append(element.P.title);
	$('#'+index).click(function (){
		$.each($('#sortable').children(),function(key,entry){
			if($(entry).hasClass('sortable-select'))
				$(entry).removeClass('sortable-select');
		});
		var ul = $(this);
		if(!ul.hasClass('sortable-select'))
			ul.addClass('sortable-select');
		$('#layer-settings').show();
		$('#layer-settings').attr('name',index);
		$('#palettes-sel').empty();
		$('#style-sel').empty();
		if(typeof layer_styles[index] === 'undefined'){
			$('#layer-settings').hide();
			return;
		}
		update_legend();
		$('#range-sels').remove();
		for(p of layer_styles[index].tile_styles.palettes){$('#palettes-sel').append($('<option>', {text: p, val: p}));};
		for(s of layer_styles[index].tile_styles.supportedStyles){$('#style-sel').append($('<option>', {text: s, val: s}));};
		if('STYLES' in layer_collection.item(index).getSource().getParams()){
			sty = layer_collection.item(index).getSource().getParams().STYLES.split('/');
			$('#palettes-sel').val(sty[1]);
			$('#style-sel').val(sty[0]);
		}else{
			$('#palettes-sel').selectedIndex = 0;
			$('#style-sel').selectedIndex = 0;
		}
		$('#opacity').val(layer_collection.item(index).getOpacity()*100);
		$('#ol-main-menu').append($('<div>', {id:'range-sels'}));
		for(e in layer_styles[index].Dimension){
			var values = layer_styles[index].Dimension[e].values.split(',');
			if(values.length > 1){
				var dname = layer_styles[index].Dimension[e].name;
				var id = dname+'-slider';
				$('#range-sels').append($('<div>', {text:'Depth (' + layer_styles[index].Dimension[e].unitSymbol + '):'}));
				$('#range-sels').append($('<div>', {text:values[0],id:'depth-display'}));
				$('#range-sels').append($('<input>', {id:id,type:'range'}));
				$('#'+id).attr('min',-values.length+1);
				$('#'+id).attr('max',0);
				$('#'+id).change(function(){
					$('#depth-display').text(values[-$('#'+id).val()]);
					layer_collection.item(index).getSource().updateParams({'ELEVATION':values[-$('#'+id).val()]});
				});
				$('#'+id).mousemove(function(){
					$('#depth-display').text(values[-$('#'+id).val()]);
					layer_collection.item(index).getSource().updateParams({'ELEVATION':values[-$('#'+id).val()]});
				});
			}
		}
	});
});


$('#ol-main-menu').append($('<div>', {id:'layer-settings'}));
$('#layer-settings').hide();

// Palettes Selector
$('#layer-settings').append('<br/>Pallete:<br/>');
$('#layer-settings').append($('<select>', {id:'palettes-sel'}));
$('#palettes-sel').change(update_sty);

// Style Selector
$('#layer-settings').append('<br/>Style:<br/>');
$('#layer-settings').append($('<select>', {id:'style-sel'}));
$('#style-sel').change(update_sty);

$('#layer-settings').append('<br/>Opacity:<br/>');
$('#layer-settings').append($('<input>', {id:'opacity',type:'range',min:0,max:100}));
$('#opacity').change(update_sty);
$('#opacity').mousemove(update_sty);

$(function(){
	$('#sortable').sortable({
		update: function(event, ui){
			$.each($('#sortable').children(),function(key,entry){
				layer_collection.item($(entry).attr('id')).setZIndex(-key);
			});
		}
	});
	$('#sortable').disableSelection();
});


$('div.map>div.ol-viewport>div.ol-overlaycontainer-stopevent').append($('<div>', {id:'legend',class:'legend'}));
$('#legend').append($('<img>'));