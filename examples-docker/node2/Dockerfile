#See https://aka.ms/customizecontainer to learn how to customize your debug container and how Visual Studio uses this Dockerfile to build your images for faster debugging.

FROM mcr.microsoft.com/dotnet/runtime:7.0 AS base
WORKDIR /app

FROM mcr.microsoft.com/dotnet/sdk:7.0 AS build
WORKDIR /src
COPY ["node2/node2.csproj", "node2/"]
RUN dotnet restore "node2/node2.csproj"
COPY . .
WORKDIR "/src/node2"
RUN dotnet build "node2.csproj" -c Release -o /app/build

FROM build AS publish
RUN dotnet publish --self-contained --runtime linux-x64 "node2.csproj" -c Release -o /app/publish

FROM ros:humble-ros-core AS final
WORKDIR /app
COPY --from=publish /app/publish .
ENTRYPOINT . /opt/ros/humble/setup.sh && ./node2