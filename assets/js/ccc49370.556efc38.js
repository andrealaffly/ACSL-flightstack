"use strict";(self.webpackChunkmy_website=self.webpackChunkmy_website||[]).push([[249],{3858:(e,t,n)=>{n.r(t),n.d(t,{default:()=>b});n(6540);var i=n(4164),a=n(1213),s=n(7559),r=n(4096),o=n(8027),l=n(2230),c=n(1312),d=n(9022),u=n(4848);function m(e){const{nextItem:t,prevItem:n}=e;return(0,u.jsxs)("nav",{className:"pagination-nav docusaurus-mt-lg","aria-label":(0,c.T)({id:"theme.blog.post.paginator.navAriaLabel",message:"Blog post page navigation",description:"The ARIA label for the blog posts pagination"}),children:[n&&(0,u.jsx)(d.A,{...n,subLabel:(0,u.jsx)(c.A,{id:"theme.blog.post.paginator.newerPost",description:"The blog post button label to navigate to the newer/previous post",children:"Newer post"})}),t&&(0,u.jsx)(d.A,{...t,subLabel:(0,u.jsx)(c.A,{id:"theme.blog.post.paginator.olderPost",description:"The blog post button label to navigate to the older/next post",children:"Older post"}),isNext:!0})]})}function f(){const{assets:e,metadata:t}=(0,r.e7)(),{title:n,description:i,date:s,tags:o,authors:l,frontMatter:c}=t,{keywords:d}=c,m=e.image??c.image;return(0,u.jsxs)(a.be,{title:c.title_meta??n,description:i,keywords:d,image:m,children:[(0,u.jsx)("meta",{property:"og:type",content:"article"}),(0,u.jsx)("meta",{property:"article:published_time",content:s}),l.some((e=>e.url))&&(0,u.jsx)("meta",{property:"article:author",content:l.map((e=>e.url)).filter(Boolean).join(",")}),o.length>0&&(0,u.jsx)("meta",{property:"article:tag",content:o.map((e=>e.label)).join(",")})]})}var h=n(5260);function g(){const e=(0,r.J_)();return(0,u.jsx)(h.A,{children:(0,u.jsx)("script",{type:"application/ld+json",children:JSON.stringify(e)})})}var p=n(7763),x=n(6896);function v(e){let{sidebar:t,children:n}=e;const{metadata:i,toc:a}=(0,r.e7)(),{nextItem:s,prevItem:c,frontMatter:d}=i,{hide_table_of_contents:f,toc_min_heading_level:h,toc_max_heading_level:g}=d;return(0,u.jsxs)(o.A,{sidebar:t,toc:!f&&a.length>0?(0,u.jsx)(p.A,{toc:a,minHeadingLevel:h,maxHeadingLevel:g}):void 0,children:[(0,u.jsx)(x.A,{metadata:i}),(0,u.jsx)(l.A,{children:n}),(s||c)&&(0,u.jsx)(m,{nextItem:s,prevItem:c})]})}function b(e){const t=e.content;return(0,u.jsx)(r.in,{content:e.content,isBlogPostPage:!0,children:(0,u.jsxs)(a.e3,{className:(0,i.A)(s.G.wrapper.blogPages,s.G.page.blogPostPage),children:[(0,u.jsx)(f,{}),(0,u.jsx)(g,{}),(0,u.jsx)(v,{sidebar:e.sidebar,children:(0,u.jsx)(t,{})})]})})}},6896:(e,t,n)=>{n.d(t,{A:()=>x});n(6540);var i=n(4164),a=n(1312),s=n(5260),r=n(4848);function o(){return(0,r.jsx)(a.A,{id:"theme.contentVisibility.unlistedBanner.title",description:"The unlisted content banner title",children:"Unlisted page"})}function l(){return(0,r.jsx)(a.A,{id:"theme.contentVisibility.unlistedBanner.message",description:"The unlisted content banner message",children:"This page is unlisted. Search engines will not index it, and only users having a direct link can access it."})}function c(){return(0,r.jsx)(s.A,{children:(0,r.jsx)("meta",{name:"robots",content:"noindex, nofollow"})})}function d(){return(0,r.jsx)(a.A,{id:"theme.contentVisibility.draftBanner.title",description:"The draft content banner title",children:"Draft page"})}function u(){return(0,r.jsx)(a.A,{id:"theme.contentVisibility.draftBanner.message",description:"The draft content banner message",children:"This page is a draft. It will only be visible in dev and be excluded from the production build."})}var m=n(7559),f=n(7293);function h(e){let{className:t}=e;return(0,r.jsx)(f.A,{type:"caution",title:(0,r.jsx)(d,{}),className:(0,i.A)(t,m.G.common.draftBanner),children:(0,r.jsx)(u,{})})}function g(e){let{className:t}=e;return(0,r.jsx)(f.A,{type:"caution",title:(0,r.jsx)(o,{}),className:(0,i.A)(t,m.G.common.unlistedBanner),children:(0,r.jsx)(l,{})})}function p(e){return(0,r.jsxs)(r.Fragment,{children:[(0,r.jsx)(c,{}),(0,r.jsx)(g,{...e})]})}function x(e){let{metadata:t}=e;const{unlisted:n,frontMatter:i}=t;return(0,r.jsxs)(r.Fragment,{children:[(n||i.unlisted)&&(0,r.jsx)(p,{}),i.draft&&(0,r.jsx)(h,{})]})}},7763:(e,t,n)=>{n.d(t,{A:()=>c});n(6540);var i=n(4164),a=n(5195);const s={tableOfContents:"tableOfContents_bqdL",docItemContainer:"docItemContainer_F8PC"};var r=n(4848);const o="table-of-contents__link toc-highlight",l="table-of-contents__link--active";function c(e){let{className:t,...n}=e;return(0,r.jsx)("div",{className:(0,i.A)(s.tableOfContents,"thin-scrollbar",t),children:(0,r.jsx)(a.A,{...n,linkClassName:o,linkActiveClassName:l})})}},5195:(e,t,n)=>{n.d(t,{A:()=>g});var i=n(6540),a=n(6342);function s(e){const t=e.map((e=>({...e,parentIndex:-1,children:[]}))),n=Array(7).fill(-1);t.forEach(((e,t)=>{const i=n.slice(2,e.level);e.parentIndex=Math.max(...i),n[e.level]=t}));const i=[];return t.forEach((e=>{const{parentIndex:n,...a}=e;n>=0?t[n].children.push(a):i.push(a)})),i}function r(e){let{toc:t,minHeadingLevel:n,maxHeadingLevel:i}=e;return t.flatMap((e=>{const t=r({toc:e.children,minHeadingLevel:n,maxHeadingLevel:i});return function(e){return e.level>=n&&e.level<=i}(e)?[{...e,children:t}]:t}))}function o(e){const t=e.getBoundingClientRect();return t.top===t.bottom?o(e.parentNode):t}function l(e,t){let{anchorTopOffset:n}=t;const i=e.find((e=>o(e).top>=n));if(i){return function(e){return e.top>0&&e.bottom<window.innerHeight/2}(o(i))?i:e[e.indexOf(i)-1]??null}return e[e.length-1]??null}function c(){const e=(0,i.useRef)(0),{navbar:{hideOnScroll:t}}=(0,a.p)();return(0,i.useEffect)((()=>{e.current=t?0:document.querySelector(".navbar").clientHeight}),[t]),e}function d(e){const t=(0,i.useRef)(void 0),n=c();(0,i.useEffect)((()=>{if(!e)return()=>{};const{linkClassName:i,linkActiveClassName:a,minHeadingLevel:s,maxHeadingLevel:r}=e;function o(){const e=function(e){return Array.from(document.getElementsByClassName(e))}(i),o=function(e){let{minHeadingLevel:t,maxHeadingLevel:n}=e;const i=[];for(let a=t;a<=n;a+=1)i.push(`h${a}.anchor`);return Array.from(document.querySelectorAll(i.join()))}({minHeadingLevel:s,maxHeadingLevel:r}),c=l(o,{anchorTopOffset:n.current}),d=e.find((e=>c&&c.id===function(e){return decodeURIComponent(e.href.substring(e.href.indexOf("#")+1))}(e)));e.forEach((e=>{!function(e,n){n?(t.current&&t.current!==e&&t.current.classList.remove(a),e.classList.add(a),t.current=e):e.classList.remove(a)}(e,e===d)}))}return document.addEventListener("scroll",o),document.addEventListener("resize",o),o(),()=>{document.removeEventListener("scroll",o),document.removeEventListener("resize",o)}}),[e,n])}var u=n(8774),m=n(4848);function f(e){let{toc:t,className:n,linkClassName:i,isChild:a}=e;return t.length?(0,m.jsx)("ul",{className:a?void 0:n,children:t.map((e=>(0,m.jsxs)("li",{children:[(0,m.jsx)(u.A,{to:`#${e.id}`,className:i??void 0,dangerouslySetInnerHTML:{__html:e.value}}),(0,m.jsx)(f,{isChild:!0,toc:e.children,className:n,linkClassName:i})]},e.id)))}):null}const h=i.memo(f);function g(e){let{toc:t,className:n="table-of-contents table-of-contents__left-border",linkClassName:o="table-of-contents__link",linkActiveClassName:l,minHeadingLevel:c,maxHeadingLevel:u,...f}=e;const g=(0,a.p)(),p=c??g.tableOfContents.minHeadingLevel,x=u??g.tableOfContents.maxHeadingLevel,v=function(e){let{toc:t,minHeadingLevel:n,maxHeadingLevel:a}=e;return(0,i.useMemo)((()=>r({toc:s(t),minHeadingLevel:n,maxHeadingLevel:a})),[t,n,a])}({toc:t,minHeadingLevel:p,maxHeadingLevel:x});return d((0,i.useMemo)((()=>{if(o&&l)return{linkClassName:o,linkActiveClassName:l,minHeadingLevel:p,maxHeadingLevel:x}}),[o,l,p,x])),(0,m.jsx)(h,{toc:v,className:n,linkClassName:o,...f})}}}]);