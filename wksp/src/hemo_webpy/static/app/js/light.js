(function() {
    'use strict';

/* Controllers */

	var controllers = angular.module('light', []);

	controllers.controller('LightCtrl', ['$scope', '$http', '$location', lightCtrl]);

  function lightCtrl($scope, $http, $location) {
  
    $scope.colors = [];

    $scope.go = function(s) {
      $location.path(s);
    }
    
    $scope.drawLine = function(x1, y1, x2, y2) {
      var c = document.getElementById("myCanvas");
      var ctx = c.getContext("2d");
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }

    $scope.setCanvasSize = function(s) {
      var c = document.getElementById("myCanvas");
      c.width = s;
      c.height = s;
    }
    
    function onLoad(){

        var httpRequest = $http({
              method: 'GET',
              url: 'http://192.168.1.14:8080/map'
          }).success(function(data, status) {

              $scope.setCanvasSize(data.size);

              $scope.data = data.lines;
              console.log($scope.data)

              angular.forEach($scope.data, function (item) {
                $scope.drawLine(item.x1, item.y1, item.x2, item.y2)
              });
          });
    };
    
    onLoad();
  }
})();  